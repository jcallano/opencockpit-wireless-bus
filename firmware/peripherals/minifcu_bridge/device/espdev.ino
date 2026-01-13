#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// --- CONFIGURACIÓN DE RED (Debe ser IDÉNTICA al Host) ---
// ¡IMPORTANTE! Aquí pones la MAC del ESP32 HOST (el que tiene el CH340)
//MAC (Desde eFuse): DC:DA:0C:5A:8E:60
uint8_t hostMacAddress[] = {0xDC, 0xDA, 0x0C, 0x5A, 0x8E, 0x60}; 

#define WIFI_CHANNEL 1 

// --- BUFFER ELÁSTICO (Misma lógica que el Host) ---
#define MAX_PAYLOAD 250
#define FLUSH_TIMEOUT_MS 2 

uint8_t tx_buffer[MAX_PAYLOAD];
size_t tx_len = 0;
unsigned long last_byte_time = 0;

// -------------------------------------------------------------------------
// 1. RECEPCIÓN ESP-NOW (Desde el Host/CH340 -> Hacia el PC)
// -------------------------------------------------------------------------
// Firma compatible con Core 3.x (esp_now_recv_info_t)
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
    // Escribimos directamente al USB Nativo para que el PC lo lea
    // Serial.write es bloqueante si el buffer USB se llena, pero es lo deseado aquí
    Serial.write(data, len);
}

// -------------------------------------------------------------------------
// 2. ENVÍO ESP-NOW (Desde el PC -> Hacia el Host/CH340)
// -------------------------------------------------------------------------
// Callback de confirmación de envío (Opcional, mantenemos vacío por velocidad)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
   // Serial.print("[DEBUG DEVICE] Estado Envio ESP-NOW: ");
   // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "EXITO (ACK Recibido)" : "FALLO (Sin respuesta)");
}
void flush_buffer() {
    if (tx_len > 0) {
        esp_now_send(hostMacAddress, tx_buffer, tx_len);
        tx_len = 0; 
    }
}

// -------------------------------------------------------------------------
// 3. SETUP
// -------------------------------------------------------------------------
void setup() {
    // Inicializamos el Serial USB Nativo
    // Al tener "USB CDC On Boot: Enabled", Serial apunta al USB-C
    Serial.begin(9600); 
    // No esperamos while(!Serial) para que arranque aunque no abras el monitor

    // === OPTIMIZACIÓN "WIRELESS AVIONICS" (Espejo exacto del Host) ===
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    
    // Stage 3: Física - Forzar OFDM (6Mbps+)
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_protocol(WIFI_IF_AP,  WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    
    // Stage 4: Canal Fijo
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    
    // Stage 2: Energía
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
        // Si falla, parpadeamos un error o reiniciamos
        return;
    }

    // Registrar Callbacks (Con casts para evitar errores de compilador estricto)
    esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

    // Registrar al Host como Peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, hostMacAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;  
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        return;
    }
}

// -------------------------------------------------------------------------
// 4. LOOP PRINCIPAL
// -------------------------------------------------------------------------
void loop() {

    // Sonda 1: Ver si llega data del PC
    /*if (Serial.available()) {
        size_t len = Serial.available();
        Serial.print("[DEBUG DEVICE] PC -> Buffer: "); 
        Serial.print(len); 
        Serial.println(" bytes");
    }*/
    // Lectura del USB Nativo (Lo que envía el simulador/PC)
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        // Lógica de llenado de buffer
        if (tx_len >= MAX_PAYLOAD) {
            flush_buffer();
        }
        tx_buffer[tx_len++] = byte;
        last_byte_time = millis();
    }

    // Flush por tiempo (para enviar comandos cortos rápidamente)
    if (tx_len > 0 && (millis() - last_byte_time > FLUSH_TIMEOUT_MS)) {
        //Serial.printf("[DEBUG DEVICE] Flush -> Aire (%d bytes)\n", tx_len);
        flush_buffer();
    }



}