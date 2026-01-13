/**
 * OpenCockpit Wireless Avionics Bus
 * ESP-NOW Coordinator Header - Node A
 *
 * Manages peer discovery, registration, and message routing
 */

#ifndef ESPNOW_COORDINATOR_H
#define ESPNOW_COORDINATOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "../../common/protocol.h"
#include "../../common/espnow_config.h"

// Coordinator states
enum CoordinatorState {
    COORD_STATE_INIT,
    COORD_STATE_DISCOVERY,
    COORD_STATE_ACTIVE,
    COORD_STATE_ERROR
};

// Queued message structure
struct QueuedMessage {
    uint8_t mac_address[6];
    uint8_t data[MAX_ESPNOW_PAYLOAD];
    size_t length;
    uint32_t timestamp;
};

// Callback types
typedef void (*HIDInputCallback)(uint8_t device_id, const uint8_t* report, uint8_t len);
typedef void (*SerialDataCallback)(const uint8_t* data, uint8_t len);
typedef void (*NodeStatusCallback)(uint8_t node_id, bool connected);

class ESPNowCoordinator {
public:
    ESPNowCoordinator();

    // Initialize the coordinator
    bool begin();

    // Main processing (call from loop or task)
    void process();

    // Send messages to nodes
    bool sendHIDOutput(uint8_t node_id, uint8_t device_id, const uint8_t* report, uint8_t len);
    bool sendSerialData(uint8_t node_id, const uint8_t* data, uint8_t len);
    bool sendMCDUDisplay(uint8_t node_id, const uint8_t* data, uint8_t len);

    // Register callbacks
    void setHIDInputCallback(HIDInputCallback callback);
    void setSerialDataCallback(SerialDataCallback callback);
    void setNodeStatusCallback(NodeStatusCallback callback);

    // Status
    CoordinatorState getState() const { return _state; }
    uint8_t getConnectedNodeCount() const;
    const PeerInfo* getPeerInfo(uint8_t node_id) const;

    // Static callbacks for ESP-NOW
    // Note: Using older ESP-NOW API signature for Arduino ESP32 Core compatibility
    static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    static void onDataReceived(const uint8_t* mac_addr, const uint8_t* data, int len);

private:
    CoordinatorState _state;
    PeerInfo _peers[MAX_PEERS];
    uint8_t _peer_count;
    uint8_t _my_mac[6];

    // FreeRTOS primitives
    QueueHandle_t _tx_queue;
    QueueHandle_t _rx_queue;
    SemaphoreHandle_t _peer_mutex;

    // Timing
    uint32_t _last_discovery_ms;
    uint32_t _last_heartbeat_ms;

    // Callbacks
    HIDInputCallback _hid_input_callback;
    SerialDataCallback _serial_data_callback;
    NodeStatusCallback _node_status_callback;

    // Internal methods
    void processDiscovery();
    void processHeartbeats();
    void processRxQueue();
    void processTxQueue();

    void handleDiscoveryResponse(const uint8_t* mac, const DiscoveryResponse* response);
    void handleRegisterAck(const uint8_t* mac, uint8_t node_id);
    void handleHeartbeat(const uint8_t* mac, const HeartbeatPayload* payload);
    void handleHIDInput(const uint8_t* mac, const HIDInputPayload* payload);
    void handleSerialData(const uint8_t* mac, const SerialDataPayload* payload);
    void handleMCDUInput(const uint8_t* mac, const MCDUInputPayload* payload);

    bool sendMessage(const uint8_t* mac, const uint8_t* data, size_t len);
    void broadcastDiscovery();
    void sendRegistration(const uint8_t* mac, uint8_t assigned_id);
    void sendHeartbeatToNode(uint8_t node_id);

    PeerInfo* findPeerByMac(const uint8_t* mac);
    PeerInfo* findPeerById(uint8_t node_id);
    uint8_t assignNodeId();
};

// Global coordinator instance
extern ESPNowCoordinator coordinator;

#endif // ESPNOW_COORDINATOR_H
