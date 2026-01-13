/**
 * OpenCockpit Wireless Avionics Bus
 * ESP-NOW Configuration Header
 *
 * Optimized ESP-NOW settings for low-latency communication (~3ms RTT)
 * Based on documented latency testing results
 */

#ifndef ESPNOW_CONFIG_H
#define ESPNOW_CONFIG_H

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// WiFi Channel (fixed to prevent scanning delays)
#define ESPNOW_WIFI_CHANNEL 1

// Heartbeat configuration
#define HEARTBEAT_INTERVAL_MS 1000
#define HEARTBEAT_TIMEOUT_MS 3000
#define MAX_MISSED_HEARTBEATS 3

// Discovery configuration
#define DISCOVERY_INTERVAL_MS 500
#define DISCOVERY_TIMEOUT_MS 5000
#define MAX_PEERS 4

// Reconnection backoff times (ms)
static const uint32_t RECONNECT_BACKOFF[] = {500, 1000, 2000, 5000};
#define MAX_RECONNECT_ATTEMPTS (sizeof(RECONNECT_BACKOFF) / sizeof(RECONNECT_BACKOFF[0]))

// Message queue sizes
#define TX_QUEUE_SIZE 32
#define RX_QUEUE_SIZE 32

// Broadcast MAC address
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/**
 * Initialize ESP-NOW with optimized settings for low latency
 * Call this after WiFi.mode() but before esp_now_init()
 *
 * This function implements the critical optimizations:
 * 1. WIFI_AP_STA mode to keep radio active
 * 2. Disable 802.11b to force OFDM (6Mbps+)
 * 3. Lock to channel 1 to prevent scanning
 * 4. Disable power saving
 *
 * @return true if initialization successful
 */
inline bool initEspNowOptimized() {
    // 1. Force Dual Mode to keep radio alive
    WiFi.mode(WIFI_AP_STA);

    // 2. CRITICAL: Ban 802.11b to prevent 1Mbps negotiation
    // Forces OFDM modulation (6Mbps minimum)
    esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (err != ESP_OK) {
        Serial.printf("Failed to set STA protocol: %d\n", err);
        return false;
    }

    err = esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (err != ESP_OK) {
        Serial.printf("Failed to set AP protocol: %d\n", err);
        return false;
    }

    // 3. Force Channel 1 (prevent scanning)
    // Must use promiscuous mode briefly to set channel
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // 4. Brute Force Power - NO SLEEP
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    // 5. Initialize ESP-NOW
    err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("ESP-NOW init failed: %d\n", err);
        return false;
    }

    return true;
}

/**
 * Add a peer to ESP-NOW
 *
 * @param mac_address 6-byte MAC address
 * @param channel WiFi channel (default: ESPNOW_WIFI_CHANNEL)
 * @param encrypt Enable encryption (default: false for minimum latency)
 * @return true if peer added successfully
 */
inline bool addEspNowPeer(const uint8_t* mac_address, uint8_t channel = ESPNOW_WIFI_CHANNEL, bool encrypt = false) {
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac_address, 6);
    peer_info.channel = channel;
    peer_info.encrypt = encrypt;
    peer_info.ifidx = WIFI_IF_STA;

    // Check if peer already exists
    if (esp_now_is_peer_exist(mac_address)) {
        return true;
    }

    esp_err_t err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK) {
        Serial.printf("Failed to add peer: %d\n", err);
        return false;
    }

    return true;
}

/**
 * Remove a peer from ESP-NOW
 *
 * @param mac_address 6-byte MAC address
 * @return true if peer removed successfully
 */
inline bool removeEspNowPeer(const uint8_t* mac_address) {
    if (!esp_now_is_peer_exist(mac_address)) {
        return true;
    }

    esp_err_t err = esp_now_del_peer(mac_address);
    return err == ESP_OK;
}

/**
 * Get the device MAC address
 *
 * @param mac_out 6-byte buffer to store MAC address
 */
inline void getDeviceMac(uint8_t* mac_out) {
    esp_read_mac(mac_out, ESP_MAC_WIFI_STA);
}

/**
 * Format MAC address as string
 *
 * @param mac 6-byte MAC address
 * @param str Output buffer (at least 18 bytes)
 */
inline void macToString(const uint8_t* mac, char* str) {
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

#endif // ESPNOW_CONFIG_H
