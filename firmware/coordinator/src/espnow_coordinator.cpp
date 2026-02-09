/**
 * OpenCockpit Wireless Avionics Bus
 * ESP-NOW Coordinator Implementation - Node A
 */

#include "include/espnow_coordinator.h"

// Global instance
ESPNowCoordinator coordinator;

// Static pointer for callbacks
static ESPNowCoordinator* g_coordinator = nullptr;

ESPNowCoordinator::ESPNowCoordinator()
    : _state(COORD_STATE_INIT)
    , _peer_count(0)
    , _connected_count(0)
    , _tx_queue(nullptr)
    , _rx_queue(nullptr)
    , _peer_mutex(nullptr)
    , _last_discovery_ms(0)
    , _last_heartbeat_ms(0)
    , _hid_input_callback(nullptr)
    , _serial_data_callback(nullptr)
    , _node_status_callback(nullptr)
    , _test_callback(nullptr)
{
    memset(_peers, 0, sizeof(_peers));
    memset(_my_mac, 0, sizeof(_my_mac));
    g_coordinator = this;
}

bool ESPNowCoordinator::begin() {
    ESPNOW_LOG_SERIAL.println("Coordinator: Initializing...");

    // Create queues
    _tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(QueuedMessage));
    _rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(QueuedMessage));
    _peer_mutex = xSemaphoreCreateMutex();

    if (!_tx_queue || !_rx_queue || !_peer_mutex) {
        ESPNOW_LOG_SERIAL.println("Coordinator: Failed to create queues");
        _state = COORD_STATE_ERROR;
        return false;
    }

    // Initialize ESP-NOW with optimized settings
    if (!initEspNowOptimized()) {
        ESPNOW_LOG_SERIAL.println("Coordinator: Failed to init ESP-NOW");
        _state = COORD_STATE_ERROR;
        return false;
    }

    // Get our MAC address
    getDeviceMac(_my_mac);
    char mac_str[18];
    macToString(_my_mac, mac_str);
    ESPNOW_LOG_SERIAL.printf("Coordinator: MAC Address: %s\n", mac_str);

    // Register callbacks
    esp_now_register_send_cb(ESPNowCoordinator::onDataSent);
    esp_now_register_recv_cb(ESPNowCoordinator::onDataReceived);

    // Add broadcast peer for discovery
    if (!addEspNowPeer(BROADCAST_MAC)) {
        ESPNOW_LOG_SERIAL.println("Coordinator: Failed to add broadcast peer");
        _state = COORD_STATE_ERROR;
        return false;
    }

    _state = COORD_STATE_DISCOVERY;
    _last_discovery_ms = millis();
    ESPNOW_LOG_SERIAL.println("Coordinator: Starting discovery...");

    return true;
}

void ESPNowCoordinator::process() {
    uint32_t now = millis();

    switch (_state) {
        case COORD_STATE_DISCOVERY:
            processDiscovery();
            // Transition to active after timeout or when nodes found
            if (now - _last_discovery_ms > DISCOVERY_TIMEOUT_MS || _peer_count > 0) {
                _state = COORD_STATE_ACTIVE;
                ESPNOW_LOG_SERIAL.printf("Coordinator: Active with %d peers\n", _peer_count);
            }
            break;

        case COORD_STATE_ACTIVE:
            processHeartbeats();
            // Continue discovery periodically for hot-plug support
            if (now - _last_discovery_ms > DISCOVERY_INTERVAL_MS * 10) {
                broadcastDiscovery();
                _last_discovery_ms = now;
            }
            break;

        case COORD_STATE_ERROR:
            // Try to recover
            vTaskDelay(pdMS_TO_TICKS(1000));
            begin();
            break;

        default:
            break;
    }

    // Always process queues
    processRxQueue();
    processTxQueue();
}

void ESPNowCoordinator::processDiscovery() {
    uint32_t now = millis();

    if (now - _last_discovery_ms > DISCOVERY_INTERVAL_MS) {
        broadcastDiscovery();
        _last_discovery_ms = now;
    }
}

void ESPNowCoordinator::processHeartbeats() {
    uint32_t now = millis();

    if (now - _last_heartbeat_ms < HEARTBEAT_INTERVAL_MS) {
        return;
    }
    _last_heartbeat_ms = now;

    xSemaphoreTake(_peer_mutex, portMAX_DELAY);

    for (int i = 0; i < MAX_PEERS; i++) {
        if (_peers[i].registered && _peers[i].connected) {
            // Check for timeout
            if (now - _peers[i].last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS) {
                ESPNOW_LOG_SERIAL.printf("Coordinator: Node %d lost\n", _peers[i].node_id);
                _peers[i].connected = false;
                if (_connected_count > 0) _connected_count--;
                if (_node_status_callback) {
                    _node_status_callback(_peers[i].node_id, false);
                }
            } else {
                // Send heartbeat
                sendHeartbeatToNode(_peers[i].node_id);
            }
        }
    }

    xSemaphoreGive(_peer_mutex);
}

void ESPNowCoordinator::processRxQueue() {
    QueuedMessage msg;

    while (xQueueReceive(_rx_queue, &msg, 0) == pdTRUE) {
        // Validate message
        if (!validateMessage(msg.data, msg.length)) {
            ESPNOW_LOG_SERIAL.println("Coordinator: Invalid message CRC");
            continue;
        }

        const MessageHeader* hdr = (const MessageHeader*)msg.data;
        const uint8_t* payload = msg.data + sizeof(MessageHeader);
        size_t payload_len = msg.length - sizeof(MessageHeader) - 1; // -1 for CRC

        switch (hdr->msg_type) {
            case MSG_DISCOVERY_RSP:
                if (payload_len >= sizeof(DiscoveryResponse)) {
                    handleDiscoveryResponse(msg.mac_address, (const DiscoveryResponse*)payload);
                }
                break;

            case MSG_REGISTER_ACK:
                handleRegisterAck(msg.mac_address, hdr->src_node);
                break;

            case MSG_HEARTBEAT:
            case MSG_HEARTBEAT_ACK: // Treat ACK as proof of life too
                if (payload_len >= sizeof(HeartbeatPayload)) {
                    handleHeartbeat(msg.mac_address, (const HeartbeatPayload*)payload);
                }
                break;

            case MSG_HID_INPUT:
                if (payload_len >= 3) { // Minimum: device_id + report_id + length
                    handleHIDInput(msg.mac_address, (const HIDInputPayload*)payload);
                }
                break;

            case MSG_SERIAL_DATA:
                if (payload_len >= 2) { // Minimum: port_id + length
                    handleSerialData(msg.mac_address, (const SerialDataPayload*)payload);
                }
                break;

            case MSG_MCDU_INPUT:
                if (payload_len >= sizeof(MCDUInputPayload)) {
                    handleMCDUInput(msg.mac_address, (const MCDUInputPayload*)payload);
                }
                break;

            case MSG_TEST_REQ:
            case MSG_TEST_RSP:
                handleTestMessage(msg.mac_address, hdr->msg_type, payload, payload_len);
                break;

            default:
                ESPNOW_LOG_SERIAL.printf("Coordinator: Unknown message type: 0x%02X\n", hdr->msg_type);
                break;
        }
    }
}

void ESPNowCoordinator::processTxQueue() {
    QueuedMessage msg;

    // Process a few messages per cycle to avoid blocking
    for (int i = 0; i < 4; i++) {
        if (xQueueReceive(_tx_queue, &msg, 0) != pdTRUE) {
            break;
        }

        esp_err_t result = esp_now_send(msg.mac_address, msg.data, msg.length);
        if (result != ESP_OK) {
            ESPNOW_LOG_SERIAL.printf("Coordinator: Send failed: %d\n", result);
        }
    }
}

void ESPNowCoordinator::handleDiscoveryResponse(const uint8_t* mac, const DiscoveryResponse* response) {
    char mac_str[18];
    macToString(mac, mac_str);
    ESPNOW_LOG_SERIAL.printf("Coordinator: Discovery response from %s (type=0x%02X caps=0x%02X)\n",
                   mac_str, response->node_type, response->capabilities);

    xSemaphoreTake(_peer_mutex, portMAX_DELAY);

    // Check if already known
    PeerInfo* peer = findPeerByMac(mac);
    if (peer == nullptr) {
        // Find empty slot
        for (int i = 0; i < MAX_PEERS; i++) {
            if (!_peers[i].registered) {
                peer = &_peers[i];
                break;
            }
        }
    }

    if (peer == nullptr) {
        ESPNOW_LOG_SERIAL.println("Coordinator: No peer slots available");
        xSemaphoreGive(_peer_mutex);
        return;
    }

    // Store peer info
    memcpy(peer->mac_address, mac, 6);
    peer->node_type = response->node_type;
    peer->capabilities = response->capabilities;
    peer->device_count = response->device_count;
    strncpy(peer->node_name, response->node_name, MAX_NODE_NAME_SIZE - 1);
    peer->node_name[MAX_NODE_NAME_SIZE - 1] = '\0';

    // Assign node ID if not already registered
    if (!peer->registered) {
        peer->node_id = assignNodeId(response->node_type, response->capabilities);
        ESPNOW_LOG_SERIAL.printf("Coordinator: Assigned node id 0x%02X to %s\n", peer->node_id, mac_str);
        _peer_count++;
    }

    bool needs_registration = !peer->registered;
    bool already_connected = peer->registered && peer->connected;
    uint8_t node_id = peer->node_id;
    xSemaphoreGive(_peer_mutex);

    // Add as ESP-NOW peer
    addEspNowPeer(mac);

    if (needs_registration) {
        // New peer: send registration
        sendRegistration(mac, node_id);
    } else if (already_connected && _node_status_callback) {
        // Already registered & connected: notify PC so DISC commands get a response
        _node_status_callback(node_id, true);
    }
}

void ESPNowCoordinator::handleRegisterAck(const uint8_t* mac, uint8_t node_id) {
    xSemaphoreTake(_peer_mutex, portMAX_DELAY);

    PeerInfo* peer = findPeerByMac(mac);
    if (peer) {
        if (peer->registered && peer->connected) {
             // Already registered, just update timestamp to avoid churn
             peer->last_heartbeat_ms = millis();
             xSemaphoreGive(_peer_mutex);
             return;
        }

        peer->registered = true;
        peer->connected = true;
        peer->last_heartbeat_ms = millis();
        _connected_count++;
        ESPNOW_LOG_SERIAL.printf("Coordinator: Register ACK from %02X (peer id %02X)\n",
                       node_id, peer->node_id);
        ESPNOW_LOG_SERIAL.printf("Coordinator: Node %d registered: %s\n", peer->node_id, peer->node_name);

        if (_node_status_callback) {
            _node_status_callback(peer->node_id, true);
        }
    }

    xSemaphoreGive(_peer_mutex);
}

void ESPNowCoordinator::handleHeartbeat(const uint8_t* mac, const HeartbeatPayload* payload) {
    xSemaphoreTake(_peer_mutex, portMAX_DELAY);

    PeerInfo* peer = findPeerByMac(mac);
    if (peer && peer->registered) {
        peer->last_heartbeat_ms = millis();

        if (!peer->connected) {
            peer->connected = true;
            _connected_count++;
            ESPNOW_LOG_SERIAL.printf("Coordinator: Node %d reconnected\n", peer->node_id);
            if (_node_status_callback) {
                _node_status_callback(peer->node_id, true);
            }
        }

        // Send heartbeat ACK
        HeartbeatPayload ack = {millis(), 0};
        uint8_t buffer[32];
        size_t len = buildMessage(buffer, MSG_HEARTBEAT_ACK, NODE_COORDINATOR,
                                  peer->node_id, &ack, sizeof(ack));
        sendMessage(mac, buffer, len);
    }

    xSemaphoreGive(_peer_mutex);
}

void ESPNowCoordinator::handleHIDInput(const uint8_t* mac, const HIDInputPayload* payload) {
    if (_hid_input_callback && payload->report_length <= MAX_HID_REPORT_SIZE) {
        uint8_t node_id = NODE_BROADCAST;
        PeerInfo* peer = findPeerByMac(mac);
        if (peer) {
            node_id = peer->node_id;
        }
        _hid_input_callback(node_id, payload->device_id, payload->report_id,
                            payload->report_data, payload->report_length);
    }
}

void ESPNowCoordinator::handleSerialData(const uint8_t* mac, const SerialDataPayload* payload) {
    if (_serial_data_callback && payload->data_length <= MAX_SERIAL_DATA_SIZE) {
        uint8_t node_id = NODE_BROADCAST;
        PeerInfo* peer = findPeerByMac(mac);
        if (peer) {
            node_id = peer->node_id;
        }
        _serial_data_callback(node_id, payload->port_id, payload->data, payload->data_length);
    }
}

void ESPNowCoordinator::handleMCDUInput(const uint8_t* mac, const MCDUInputPayload* payload) {
    // MCDU button press - route to HID callback with MCDU device ID
    if (_hid_input_callback) {
        uint8_t node_id = NODE_BROADCAST;
        PeerInfo* peer = findPeerByMac(mac);
        if (peer) {
            node_id = peer->node_id;
        }
        uint8_t report[2] = {payload->button_index, payload->button_state};
        _hid_input_callback(node_id, DEV_WINWING_MCDU, 0, report, 2);
    }
}

bool ESPNowCoordinator::sendHIDOutput(uint8_t node_id, uint8_t device_id,
                                       const uint8_t* report, uint8_t len) {
    xSemaphoreTake(_peer_mutex, portMAX_DELAY);
    PeerInfo* peer = findPeerById(node_id);
    if (!peer || !peer->connected) {
        xSemaphoreGive(_peer_mutex);
        return false;
    }

    uint8_t mac[6];
    memcpy(mac, peer->mac_address, 6);
    xSemaphoreGive(_peer_mutex);

    HIDOutputPayload payload;
    payload.device_id = device_id;
    payload.report_id = 0;
    payload.report_length = len;
    memcpy(payload.report_data, report, len);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t msg_len = buildMessage(buffer, MSG_HID_OUTPUT, NODE_COORDINATOR, node_id,
                                   &payload, 3 + len);

    return sendMessage(mac, buffer, msg_len);
}

bool ESPNowCoordinator::sendSerialData(uint8_t node_id, uint8_t port_id,
                                       const uint8_t* data, uint8_t len) {
    xSemaphoreTake(_peer_mutex, portMAX_DELAY);
    PeerInfo* peer = findPeerById(node_id);
    if (!peer || !peer->connected) {
        xSemaphoreGive(_peer_mutex);
        return false;
    }

    uint8_t mac[6];
    memcpy(mac, peer->mac_address, 6);
    xSemaphoreGive(_peer_mutex);

    SerialDataPayload payload;
    payload.port_id = port_id;
    payload.data_length = len;
    memcpy(payload.data, data, len);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t msg_len = buildMessage(buffer, MSG_SERIAL_DATA, NODE_COORDINATOR, node_id,
                                   &payload, 2 + len);

    return sendMessage(mac, buffer, msg_len);
}

bool ESPNowCoordinator::sendMCDUDisplay(uint8_t node_id, const uint8_t* data, uint8_t len) {
    xSemaphoreTake(_peer_mutex, portMAX_DELAY);
    PeerInfo* peer = findPeerById(node_id);
    if (!peer || !peer->connected) {
        xSemaphoreGive(_peer_mutex);
        return false;
    }

    uint8_t mac[6];
    memcpy(mac, peer->mac_address, 6);
    xSemaphoreGive(_peer_mutex);

    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
    size_t msg_len = buildMessage(buffer, MSG_MCDU_DISPLAY, NODE_COORDINATOR, node_id,
                                   data, len);

    return sendMessage(mac, buffer, msg_len);
}

void ESPNowCoordinator::setHIDInputCallback(HIDInputCallback callback) {
    _hid_input_callback = callback;
}

void ESPNowCoordinator::setSerialDataCallback(SerialDataCallback callback) {
    _serial_data_callback = callback;
}

void ESPNowCoordinator::setNodeStatusCallback(NodeStatusCallback callback) {
    _node_status_callback = callback;
}

void ESPNowCoordinator::setTestMessageCallback(TestMessageCallback callback) {
    _test_callback = callback;
}

uint8_t ESPNowCoordinator::getConnectedNodeCount() const {
    return _connected_count;
}

void ESPNowCoordinator::handleTestMessage(const uint8_t* mac, uint8_t msg_type,
                                          const uint8_t* payload, size_t payload_len) {
    if (!_test_callback) {
        return;
    }
    uint8_t node_id = NODE_BROADCAST;
    PeerInfo* peer = findPeerByMac(mac);
    if (peer) {
        node_id = peer->node_id;
    }
    _test_callback(msg_type, node_id, payload, payload_len);
}

const PeerInfo* ESPNowCoordinator::getPeerInfo(uint8_t node_id) const {
    for (int i = 0; i < MAX_PEERS; i++) {
        if (_peers[i].node_id == node_id && _peers[i].registered) {
            return &_peers[i];
        }
    }
    return nullptr;
}

bool ESPNowCoordinator::sendMessage(const uint8_t* mac, const uint8_t* data, size_t len) {
    QueuedMessage msg;
    memcpy(msg.mac_address, mac, 6);
    memcpy(msg.data, data, len);
    msg.length = len;
    msg.timestamp = millis();

    return xQueueSend(_tx_queue, &msg, pdMS_TO_TICKS(10)) == pdTRUE;
}

void ESPNowCoordinator::broadcastDiscovery() {
    uint8_t buffer[32];
    size_t len = buildMessage(buffer, MSG_DISCOVERY_REQ, NODE_COORDINATOR,
                              NODE_BROADCAST, nullptr, 0);

    esp_now_send(BROADCAST_MAC, buffer, len);
}

void ESPNowCoordinator::sendRegistration(const uint8_t* mac, uint8_t assigned_id) {
    RegisterRequest req;
    req.assigned_node_id = assigned_id;
    req.poll_interval_ms = 8; // 125Hz
    req.reserved[0] = 0;
    req.reserved[1] = 0;

    uint8_t buffer[32];
    size_t len = buildMessage(buffer, MSG_REGISTER_REQ, NODE_COORDINATOR,
                              assigned_id, &req, sizeof(req));

    char mac_str[18];
    macToString(mac, mac_str);
    ESPNOW_LOG_SERIAL.printf("Coordinator: Send REGISTER_REQ id=0x%02X -> %s\n", assigned_id, mac_str);
    esp_now_send(mac, buffer, len);
}

void ESPNowCoordinator::sendHeartbeatToNode(uint8_t node_id) {
    PeerInfo* peer = findPeerById(node_id);
    if (!peer) return;

    HeartbeatPayload payload = {millis(), 0};
    uint8_t buffer[32];
    size_t len = buildMessage(buffer, MSG_HEARTBEAT, NODE_COORDINATOR,
                              node_id, &payload, sizeof(payload));

    sendMessage(peer->mac_address, buffer, len);
}

PeerInfo* ESPNowCoordinator::findPeerByMac(const uint8_t* mac) {
    for (int i = 0; i < MAX_PEERS; i++) {
        if (memcmp(_peers[i].mac_address, mac, 6) == 0) {
            return &_peers[i];
        }
    }
    return nullptr;
}

PeerInfo* ESPNowCoordinator::findPeerById(uint8_t node_id) {
    for (int i = 0; i < MAX_PEERS; i++) {
        if (_peers[i].node_id == node_id && _peers[i].registered) {
            return &_peers[i];
        }
    }
    return nullptr;
}

uint8_t ESPNowCoordinator::assignNodeId(uint8_t node_type, uint8_t capabilities) {
    auto is_used = [this](uint8_t id) {
        for (int i = 0; i < MAX_PEERS; i++) {
            if (_peers[i].registered && _peers[i].node_id == id) {
                return true;
            }
        }
        return false;
    };

    uint8_t preferred = NODE_BROADCAST;
    if ((capabilities & CAP_HID_OUTPUT) && (capabilities & CAP_DISPLAY)) {
        // Bidirectional HID with display (e.g. Ursa Minor throttle)
        preferred = NODE_D_THROTTLE;
    } else if ((capabilities & CAP_SERIAL) && !(capabilities & CAP_DISPLAY)) {
        preferred = NODE_C_QUADRANT;
    } else if (capabilities & CAP_DISPLAY) {
        preferred = NODE_B_JOYSTICK;
    } else if (node_type == NODE_TYPE_SERIAL) {
        preferred = NODE_C_QUADRANT;
    } else if (node_type == NODE_TYPE_HID) {
        preferred = NODE_B_JOYSTICK;
    }

    if (preferred != NODE_BROADCAST && !is_used(preferred)) {
        return preferred;
    }

    for (uint8_t id = NODE_B_JOYSTICK; id < NODE_BROADCAST; id++) {
        if (!is_used(id)) {
            return id;
        }
    }
    return NODE_BROADCAST; // Error: no IDs available
}

// Static ESP-NOW callbacks
void ESPNowCoordinator::onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
    (void)info;
    if (status != ESP_NOW_SEND_SUCCESS) {
        // Could implement retry logic here
    }
}

void ESPNowCoordinator::onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (!g_coordinator || !info || !data || len <= 0 || len > MAX_ESPNOW_PAYLOAD) return;

    QueuedMessage msg;
    memcpy(msg.mac_address, info->src_addr, 6);
    memcpy(msg.data, data, len);
    msg.length = len;
    msg.timestamp = millis();

    // Don't block in ISR context
    xQueueSendFromISR(g_coordinator->_rx_queue, &msg, nullptr);
}
