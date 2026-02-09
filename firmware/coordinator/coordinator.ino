/**
 * OpenCockpit Wireless Avionics Bus
 * Node A - Coordinator (UART0 + SLIP)
 *
 * - UART0 (SLIP framing) to PC
 * - ESP-NOW coordinator to peripheral nodes
 * - LCD metrics (messages/sec)
 */

#include <Arduino.h>
#include <SPI.h>
#include "USB.h"
// #include "USBCDC.h" // Removed for CDC_ON_BOOT=1

struct NullSerial {
    template <typename... Args>
    void printf(const char*, Args...) {}
    void println(const char*) {}
};
static NullSerial NullSerialInstance;
#define DEBUG_SLIP 0
#include "include/slip_config.h"

#if DEBUG_SLIP
#define SLIP_LOG_SERIAL Serial0
#else
#define SLIP_LOG_SERIAL NullSerialInstance
#endif

#if SLIP_USE_CDC
// FORCE Debug to Serial0 (UART) if SLIP uses CDC
#define ESPNOW_LOG_SERIAL Serial0
#else
#define ESPNOW_LOG_SERIAL Serial
#endif

#include "../common/slip.h"
#include "include/espnow_coordinator.h"

// ------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------
static void process_slip_rx();
static void send_to_pc(uint8_t msg_type, uint8_t src_node, const void* payload, size_t payload_len);
static void onHIDInput(uint8_t node_id, uint8_t device_id, uint8_t report_id,
                       const uint8_t* report, uint8_t len);
static void onSerialData(uint8_t node_id, uint8_t port_id, const uint8_t* data, uint8_t len);
static void onNodeStatus(uint8_t node_id, bool connected);
static void onTestMessage(uint8_t msg_type, uint8_t node_id,
                          const uint8_t* data, uint8_t len);
static void espnow_task(void* parameter);

static void log_uart_status();
static bool slip_send_frame(const uint8_t* data, size_t len);

#define BOARD_ESPRESSIF_USB_OTG
#include "../common/hardware_config.h"

// ------------------------------------------------------------
// LCD parameters (ST7789, 240x240) - Adafruit GFX
// ------------------------------------------------------------
// BitOrder Fix for ESP32 Core v3 + BusIO
#if defined(ESP32) && (ARDUINO_ESP32_MAJOR >= 3)
  #ifndef BitOrder
    typedef uint8_t BitOrder;
  #endif
#endif

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define LCD_WIDTH           240
#define LCD_HEIGHT          240

// Colors
#define COLOR_BLACK         ST77XX_BLACK
#define COLOR_WHITE         ST77XX_WHITE
#define COLOR_GREEN         ST77XX_GREEN
#define COLOR_CYAN          ST77XX_CYAN
#define COLOR_YELLOW        ST77XX_YELLOW
#define COLOR_RED           ST77XX_RED
#define COLOR_BLUE          ST77XX_BLUE

// Initialize with SPI pins
// Adafruit_ST7789(SPIClass *spiClass, int8_t cs, int8_t dc, int8_t rst);
Adafruit_ST7789 tft = Adafruit_ST7789(&SPI, PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RST);

// ------------------------------------------------------------
// SLIP transport
// ------------------------------------------------------------
#if SLIP_USE_CDC
static Stream& slipSerial = Serial; // Serial is now USB CDC
#else
static Stream& slipSerial = Serial0;
#endif
static SlipDecoder slip_decoder;

// ------------------------------------------------------------
// Metrics
// ------------------------------------------------------------
static volatile uint32_t g_slip_rx_frames = 0;
static volatile uint32_t g_slip_tx_frames = 0;
static volatile uint32_t g_esp_rx_frames = 0;
static volatile uint32_t g_esp_tx_frames = 0;
static volatile uint32_t g_esp_rx_bytes = 0;
static volatile uint32_t g_esp_tx_bytes = 0;

static uint32_t g_latency_sum_ms = 0;
static uint32_t g_latency_samples = 0;
static uint32_t g_jitter_max_ms = 0;
static uint32_t g_last_rtt_ms = 0;
static bool g_has_last_rtt = false;

static uint32_t g_test_seq = 0;
static uint32_t g_test_last_send_ms = 0;
static bool g_test_inflight = false;
static uint8_t g_latency_target_node = NODE_BROADCAST;

#define ESPNOW_LINK_BPS 6000000UL
#define TEST_PING_INTERVAL_MS 500
#define TEST_PING_TIMEOUT_MS 250

// ------------------------------------------------------------
// LCD functions (Wrapped to GFX)
// ------------------------------------------------------------
static void lcd_init() {
    setupHardware();
    
    // SPI Init with explicit pins
    // SPI.begin(SCK, MISO, MOSI, SS);
    SPI.begin(PIN_LCD_CLK, -1, PIN_LCD_MOSI, PIN_LCD_CS);
    
    // Backlight - Explicitly ON
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);

    // GFX Init
    tft.init(240, 240);
    tft.setRotation(2); // 180 degree rotation often fits USB-OTG board
    tft.fillScreen(COLOR_BLACK);
    
    // Startup Test
    tft.fillScreen(COLOR_RED);
    delay(500);
    tft.fillScreen(COLOR_GREEN);
    delay(500);
    tft.fillScreen(COLOR_BLACK);
    
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.print("OC WIRELESS");
}

static void lcd_fill_screen(uint16_t color) {
    tft.fillScreen(color);
}

static void lcd_draw_text(uint16_t x, uint16_t y, const char* text, uint16_t color, uint16_t bg, uint8_t scale) {
    tft.setCursor(x, y);
    tft.setTextColor(color, bg);
    tft.setTextSize(scale);
    tft.print(text);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    tft.fillRect(x, y, w, h, color);
}

static void lcd_update_line(uint8_t index, const char* text, uint16_t color) {
     static char last_lines[8][32] = {};
     static const uint16_t kLineHeight = 20;
     static const uint16_t kStartY = 35;

     if (index >= 8) return;

     if (strncmp(last_lines[index], text, sizeof(last_lines[index]) - 1) == 0) {
         return;
     }

     uint16_t y = kStartY + index * kLineHeight;
     tft.fillRect(0, y, LCD_WIDTH, kLineHeight, COLOR_BLACK);
     
     tft.setCursor(10, y);
     tft.setTextColor(color);
     tft.setTextSize(2);
     tft.print(text);

     strncpy(last_lines[index], text, sizeof(last_lines[index]) - 1);
     last_lines[index][sizeof(last_lines[index]) - 1] = '\0';
}

// ------------------------------------------------------------
// SLIP -> ESP-NOW routing
// ------------------------------------------------------------
static void handle_slip_frame(const uint8_t* data, size_t len) {
    if (!validateMessage(data, len)) {
#if DEBUG_SLIP
        SLIP_LOG_SERIAL.println("SLIP: invalid frame");
#endif
        return;
    }

    const MessageHeader* hdr = reinterpret_cast<const MessageHeader*>(data);
    const uint8_t* payload = data + sizeof(MessageHeader);
    size_t payload_len = len - sizeof(MessageHeader) - 1;
#if DEBUG_SLIP
    SLIP_LOG_SERIAL.printf("SLIP: msg=0x%02X src=0x%02X dst=0x%02X len=%u\n",
                   hdr->msg_type, hdr->src_node, hdr->dst_node,
                   static_cast<unsigned>(len));
#endif

    if (hdr->dst_node == NODE_COORDINATOR && hdr->msg_type == MSG_RESET) {
        ESP.restart();
        return;
    }

    switch (hdr->msg_type) {
        case MSG_HID_OUTPUT: {
            if (payload_len < 3) break;
            const HIDOutputPayload* out = reinterpret_cast<const HIDOutputPayload*>(payload);
            if (payload_len < static_cast<size_t>(3 + out->report_length)) break;
            if (coordinator.sendHIDOutput(hdr->dst_node, out->device_id,
                                          out->report_data, out->report_length)) {
                g_esp_tx_frames++;
                g_esp_tx_bytes += sizeof(MessageHeader) + 3 + out->report_length + 1;
            }
            break;
        }
        case MSG_SERIAL_DATA: {
            if (payload_len < 2) break;
            const SerialDataPayload* serial = reinterpret_cast<const SerialDataPayload*>(payload);
            if (payload_len < static_cast<size_t>(2 + serial->data_length)) break;
            if (coordinator.sendSerialData(hdr->dst_node, serial->port_id,
                                           serial->data, serial->data_length)) {
                g_esp_tx_frames++;
                g_esp_tx_bytes += sizeof(MessageHeader) + 2 + serial->data_length + 1;
            }
            break;
        }
        case MSG_MCDU_DISPLAY: {
            if (payload_len == 0) break;
            if (coordinator.sendMCDUDisplay(hdr->dst_node, payload,
                                            static_cast<uint8_t>(payload_len))) {
                g_esp_tx_frames++;
                g_esp_tx_bytes += sizeof(MessageHeader) + payload_len + 1;
            }
            break;
        }
        case MSG_TEST_REQ: {
            if (hdr->dst_node == NODE_COORDINATOR) {
                send_to_pc(MSG_TEST_RSP, NODE_COORDINATOR, payload, payload_len);
            } else {
                const PeerInfo* peer = coordinator.getPeerInfo(hdr->dst_node);
                if (peer) {
                    uint8_t buffer[MAX_ESPNOW_PAYLOAD];
                    size_t msg_len = buildMessage(buffer, MSG_TEST_REQ, NODE_COORDINATOR,
                                                  hdr->dst_node, payload, payload_len);
                    esp_now_send(peer->mac_address, buffer, msg_len);
                    g_esp_tx_frames++;
                    g_esp_tx_bytes += msg_len;
                }
            }
            break;
        }
        case MSG_DISCOVERY_REQ: {
            // Found bug: Request was ignored. Force immediately.
            coordinator.broadcastDiscovery(); 
            /*
            for (uint8_t id = NODE_B_JOYSTICK; id < NODE_BROADCAST; id++) {
                const PeerInfo* peer = coordinator.getPeerInfo(id);
                if (!peer) continue;
                // Currently discovery is broadcast, can implement directed discovery here
            }
            */
            break;
        }
        default: break;
    }
}

static void process_slip_rx() {
    while (slipSerial.available()) {
        uint8_t byte = slipSerial.read();
        if (slip_decode_byte(&slip_decoder, byte)) {
            // Frame complete
            handle_slip_frame(slip_decoder.buffer, slip_decoder.index);
            slip_decoder_reset(&slip_decoder);
            g_slip_rx_frames++;
        }
    }
}

// ------------------------------------------------------------
// ESP-NOW Callbacks
// ------------------------------------------------------------
static void send_to_pc(uint8_t msg_type, uint8_t src_node, const void* payload, size_t payload_len) {
    uint8_t buffer[SLIP_MAX_FRAME_SIZE];
    size_t msg_len = buildMessage(buffer, static_cast<MessageType>(msg_type), src_node, NODE_COORDINATOR,
                                  payload, payload_len);
    slip_send_frame(buffer, msg_len);
    g_slip_tx_frames++;
}

static void onHIDInput(uint8_t node_id, uint8_t device_id, uint8_t report_id,
                       const uint8_t* report, uint8_t len) {
    HIDInputPayload hid;
    hid.device_id = device_id;
    hid.report_id = report_id;
    hid.report_length = len;
    memcpy(hid.report_data, report, len);
    send_to_pc(MSG_HID_INPUT, node_id, &hid, 3 + len);
}

static void onSerialData(uint8_t node_id, uint8_t port_id, const uint8_t* data, uint8_t len) {
    SerialDataPayload ser;
    ser.port_id = port_id;
    ser.data_length = len;
    memcpy(ser.data, data, len);
    send_to_pc(MSG_SERIAL_DATA, node_id, &ser, 2 + len);
}

static void onNodeStatus(uint8_t node_id, bool connected) {
    ESPNOW_LOG_SERIAL.printf("Coordinator: Node %d %s\n", node_id, connected ? "connected" : "lost");
    
    char status_line[32];
    snprintf(status_line, sizeof(status_line), "Node %d: %s", 
             node_id, connected ? "OK" : "LOST");
    uint8_t line_idx = node_id; 
    lcd_update_line(line_idx, status_line, connected ? COLOR_GREEN : COLOR_RED);

    if (connected && node_id != NODE_BROADCAST) {
        // Enable pings to this node
        if (g_latency_target_node == NODE_BROADCAST) {
            g_latency_target_node = node_id;
        }
        send_to_pc(MSG_DISCOVERY_RSP, node_id, nullptr, 0); 
    } else if (!connected && node_id == g_latency_target_node) {
        g_latency_target_node = NODE_BROADCAST; // Stop checking this node
    }
}

static void onTestMessage(uint8_t msg_type, uint8_t node_id,
                          const uint8_t* data, uint8_t len) {
    if (msg_type == MSG_TEST_RSP) {
        g_has_last_rtt = true;
        g_last_rtt_ms = millis() - g_test_last_send_ms;
        g_latency_sum_ms += g_last_rtt_ms;
        g_latency_samples++;

        // Compute jitter as max |rtt - running_avg|
        if (g_latency_samples > 1) {
            uint32_t avg = g_latency_sum_ms / g_latency_samples;
            uint32_t delta = (g_last_rtt_ms > avg) ? (g_last_rtt_ms - avg) : (avg - g_last_rtt_ms);
            if (delta > g_jitter_max_ms) g_jitter_max_ms = delta;
        }

        static uint32_t last_log = 0;
        if (millis() - last_log > 1000) {
            ESPNOW_LOG_SERIAL.printf("TEST: RTT=%u ms\n", g_last_rtt_ms);
            last_log = millis();
        }
        
        g_test_inflight = false;
        send_to_pc(MSG_TEST_RSP, node_id, data, len);
    }
}

// ------------------------------------------------------------
// Setup & Loop
// ------------------------------------------------------------
void setup() {
    // 1. Init Serial (USB CDC for SLIP)
    Serial.begin(115200); 
    
    // 2. Init Serial0 (Debug on UART0)
    Serial0.begin(115200);
    // Note: Serial0 pins are default for the board (usually 43/44 on S3 DevKit)

    
    // Init SLIP Decoder
    
    // Init SLIP Decoder
    slip_decoder_init(&slip_decoder);

    // 3. Init hardware (Backlight, etc.)
    lcd_init();

    // 4. Init ESP-NOW
    if (!coordinator.begin()) {
        ESPNOW_LOG_SERIAL.println("Coordinator: ESP-NOW Init Failed");
        lcd_draw_text(10, 100, "ESPNOW FAIL", COLOR_RED, COLOR_BLACK, 2);
        while(1) delay(100);
    }

    // Register Callbacks
    coordinator.setHIDInputCallback(onHIDInput);
    coordinator.setSerialDataCallback(onSerialData);
    coordinator.setNodeStatusCallback(onNodeStatus);
    coordinator.setTestMessageCallback(onTestMessage);

    // Initial Screen
    lcd_fill_screen(COLOR_BLACK);
    lcd_draw_text(5, 5, "OC WIRELESS BUS", COLOR_CYAN, COLOR_BLACK, 2);
    lcd_draw_text(5, 230, "Waiting for peers...", COLOR_YELLOW, COLOR_BLACK, 1);
    
    // Test Task
    xTaskCreate(espnow_task, "espnow_task", 4096, NULL, 1, NULL);
}

void loop() {
    coordinator.process();
    process_slip_rx();

    // Metrics update
    static uint32_t last_metric = 0;
    if (millis() - last_metric > 1000) {
        char buf[32];
        
        // Line 4: SLIP Stats
        snprintf(buf, sizeof(buf), "SLIP: RX%u TX%u", g_slip_rx_frames, g_slip_tx_frames);
        lcd_update_line(4, buf, COLOR_CYAN);

        // Line 5: ESP Stats
        snprintf(buf, sizeof(buf), "ESP: RX%u TX%u", g_esp_rx_frames, g_esp_tx_frames);
        lcd_update_line(5, buf, COLOR_YELLOW);
        
        // Line 6: Latency
        uint32_t rtt_avg = g_latency_samples > 0 ? (g_latency_sum_ms / g_latency_samples) : 0;
        int32_t jitter = 0; // TODO: Calculate real jitter
        if (g_has_last_rtt && g_latency_samples > 1) {
             // Simple jitter estimate: |last - avg| or store max delta?
             // Using stored max for this second
             jitter = g_jitter_max_ms; 
        }
        snprintf(buf, sizeof(buf), "RTT:%ums Jit:%ums", rtt_avg, jitter);
        lcd_update_line(6, buf, COLOR_GREEN);

        // Line 7: Nodes & Channel
        // Chan Util: (Bytes * 8 * 100) / 6Mbps
        uint32_t total_bytes = g_esp_rx_bytes + g_esp_tx_bytes;
        uint32_t chan_util = (total_bytes * 8 * 100) / 6000000;
        snprintf(buf, sizeof(buf), "Nodes:%u Ch:%u%%", coordinator.getConnectedNodeCount(), chan_util);
        lcd_update_line(7, buf, COLOR_WHITE);

        // Reset per-second counters
        g_slip_rx_frames = 0; g_slip_tx_frames = 0;
        g_esp_rx_frames = 0; g_esp_tx_frames = 0;
        g_esp_rx_bytes = 0; g_esp_tx_bytes = 0;
        g_latency_sum_ms = 0; g_latency_samples = 0;
        g_jitter_max_ms = 0;
        last_metric = millis();
    }
    
    
}

static void espnow_task(void* parameter) {
    while(1) {
        // Send Ping if configured
        if (g_latency_target_node != NODE_BROADCAST && !g_test_inflight &&
            (millis() - g_test_last_send_ms > TEST_PING_INTERVAL_MS)) {
                
            const PeerInfo* peer = coordinator.getPeerInfo(g_latency_target_node);
            if (peer) {
                uint8_t payload[4];
                memcpy(payload, &g_test_seq, 4);
                
                uint8_t buffer[MAX_ESPNOW_PAYLOAD];
                size_t msg_len = buildMessage(buffer, MSG_TEST_REQ, NODE_COORDINATOR,
                                              g_latency_target_node, payload, 4);
                
                if (esp_now_send(peer->mac_address, buffer, msg_len) == ESP_OK) {
                    g_test_last_send_ms = millis();
                    g_test_inflight = true;
                    g_test_seq++;
                }
            }
        }
        
        // Timeout check
        if (g_test_inflight && (millis() - g_test_last_send_ms > TEST_PING_TIMEOUT_MS)) {
            g_test_inflight = false; // Timeout
        }
        
        delay(10);
    }
}

static void log_uart_status() {
    (void)slipSerial;
}

static bool slip_send_frame(const uint8_t* data, size_t len) {
    size_t encoded_len = slip_encoded_size(data, len);
    static uint8_t encoded[SLIP_MAX_FRAME_SIZE * 2 + 2];
    if (encoded_len > sizeof(encoded)) {
        ESPNOW_LOG_SERIAL.println("SLIP TX FAIL: Too large");
        return false;
    }

    slip_encode(data, len, encoded);

    // Write with brief retry instead of silent drop
    for (int attempt = 0; attempt < 3; attempt++) {
        if (slipSerial.availableForWrite() >= static_cast<int>(encoded_len)) {
            size_t written = slipSerial.write(encoded, encoded_len);
            return written == encoded_len;
        }
        delayMicroseconds(500);  // 0.5ms wait between retries
    }

    // Last resort: write anyway (may block briefly but won't lose data)
    size_t written = slipSerial.write(encoded, encoded_len);
    return written == encoded_len;
}
