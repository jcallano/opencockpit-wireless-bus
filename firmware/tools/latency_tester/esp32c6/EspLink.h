#ifndef ESP_LINK_H
#define ESP_LINK_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> 

enum MsgType {
  TYPE_DATA = 0,
  TYPE_PING = 1,
  TYPE_PONG = 2
};

typedef struct {
  uint8_t type;
  char payload[200]; 
  uint32_t id;       
} Packet;

class EspLink {
  private:
    uint8_t _targetMac[6];
    esp_now_peer_info_t _peerInfo;
    
    unsigned long _startMicros;
    unsigned long _lastLatency;
    
    static EspLink* instance;

    static void _onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
      if (instance) instance->handleSent(status);
    }

    static void _onDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
      if (instance) instance->handleRecv(info, data, len);
    }

    void handleSent(esp_now_send_status_t status) { }

    void handleRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
      // Validación de seguridad básica
      if (len < 1) return;

      // Leemos solo el primer byte para saber el tipo (Evitamos copiar todo si no es necesario)
      uint8_t type = data[0];

      if (type == TYPE_PING) {
        // OPTIMIZACIÓN: Responder con un paquete PONG MINÚSCULO (solo 1 byte de cabecera)
        // No necesitamos payload para medir tiempo.
        uint8_t pongHeader = TYPE_PONG;
        esp_now_send(info->src_addr, &pongHeader, 1);
      } 
      else if (type == TYPE_PONG) {
        unsigned long endMicros = micros();
        _lastLatency = endMicros - _startMicros;
        if (onLatencyAvailable) onLatencyAvailable(_lastLatency);
      } 
      else if (type == TYPE_DATA) {
        // Solo copiamos la estructura completa si es un mensaje de datos real
        if (len > sizeof(Packet)) return;
        Packet incoming;
        memcpy(&incoming, data, len);
        if (onDataReceived) onDataReceived(incoming.payload);
      }
    }

  public:
    void (*onDataReceived)(const char* message) = nullptr;
    void (*onLatencyAvailable)(unsigned long latencyMicros) = nullptr;

    EspLink() {
      instance = this;
      _lastLatency = 0;
    }

    void begin(const uint8_t* targetMac) {
      memcpy(_targetMac, targetMac, 6);

      WiFi.mode(WIFI_AP_STA);
      
      // OPTIMIZACIÓN 1: PROHIBIR PROTOCOLO 11B (1 Mbps)
      // Eliminamos WIFI_PROTOCOL_11B de la lista. 
      // Esto fuerza a usar OFDM (Mínimo 6 Mbps, idealmente 54 Mbps o MCS7)
      esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
      esp_wifi_set_protocol(WIFI_IF_AP,  WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

      WiFi.disconnect();
      
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);

      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);

      if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        return;
      }

      esp_now_register_send_cb((esp_now_send_cb_t)_onDataSent);
      esp_now_register_recv_cb(_onDataRecv);

      memset(&_peerInfo, 0, sizeof(_peerInfo));
      memcpy(_peerInfo.peer_addr, _targetMac, 6);
      _peerInfo.channel = 1; 
      _peerInfo.encrypt = false;
      _peerInfo.ifidx = WIFI_IF_STA; 

      if (!esp_now_is_peer_exist(_targetMac)) {
        esp_now_add_peer(&_peerInfo);
      }
    }

    bool send(const char* message) {
      Packet pkt;
      pkt.type = TYPE_DATA;
      strncpy(pkt.payload, message, sizeof(pkt.payload) - 1);
      
      // OPTIMIZACIÓN 2: Enviar solo la longitud real usada, no los 200 bytes de basura
      // Tamaño = 1 byte (type) + longitud del string + 1 (null terminator) + 4 bytes (id)
      size_t realLen = 1 + strlen(pkt.payload) + 1 + 4; 
      
      esp_err_t result = esp_now_send(_targetMac, (uint8_t *) &pkt, realLen);
      return (result == ESP_OK);
    }

    void measureLatency() {
      Packet pkt;
      pkt.type = TYPE_PING;
      pkt.payload[0] = 0; 
      
      _startMicros = micros(); 
      // OPTIMIZACIÓN 3: Enviar PING MINÚSCULO
      // Solo enviamos 1 byte (el tipo). No hace falta mandar el payload vacío.
      esp_now_send(_targetMac, (uint8_t *) &pkt, 1);
    }

    unsigned long getLastLatency() {
      return _lastLatency;
    }
};

EspLink* EspLink::instance = nullptr;

#endif