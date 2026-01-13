#include "usb/usb_host.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// --- CONFIGURACIÓN DE PINES (WeAct ESP32-S3) ---
#define RXD0 44
#define TXD0 43

// --- CONFIGURACIÓN DE RED (Latencia Mínima) ---
//MAC (Desde eFuse): DC:DA:0C:5A:83:64
uint8_t peerAddress[] = {0xDC, 0xDA, 0x0C, 0x5A, 0x83, 0x64}; // ¡PONER MAC DEL DEVICE AQUÍ!
#define WIFI_CHANNEL 1 

// --- VARIABLES DE BUFFER (Wireless Avionics) ---
#define MAX_PAYLOAD 250
#define FLUSH_TIMEOUT_MS 2 

uint8_t tx_buffer[MAX_PAYLOAD];
size_t tx_len = 0;
unsigned long last_byte_time = 0;

// --- VARIABLES USB HOST (Referencias de usbothtest1.ino) ---
usb_host_client_handle_t client_hdl = NULL;
usb_device_handle_t ch340_dev_hdl = NULL;
usb_transfer_t *ctrl_xfer = NULL, *in_xfer = NULL, *out_xfer = NULL;
volatile bool ctrl_done = true;


// -------------------------------------------------------------------------
// 1. LÓGICA ESP-NOW OPTIMIZADA (Core 3.3.5 Compatible)
// -------------------------------------------------------------------------

// Corrección para Core 3.x: El primer argumento es wifi_tx_info_t, no uint8_t*
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    // Callback vacío para máxima velocidad
}

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
    // 1. Debug (Para confirmar que sigue llegando)
    Serial0.printf("WiFi -> USB: %d bytes\n", len);

    // 2. Verificaciones de seguridad
    if (!ch340_dev_hdl || !out_xfer) return; // Si no hay USB conectado, salir
    if (len > 64) len = 64; // Protección contra desbordamiento (CH340 usa paquetes de 32/64 max)

    // 3. Preparar la transferencia USB
    memcpy(out_xfer->data_buffer, data, len); // Copiar datos del WiFi al buffer USB
    out_xfer->num_bytes = len;
    out_xfer->device_handle = ch340_dev_hdl;
    out_xfer->bEndpointAddress = 0x02; // Endpoint OUT del CH340 (Casi siempre es 0x02)
    out_xfer->callback = [](usb_transfer_t *t){ 
        // Callback vacío: solo nos interesa que se envió
    };
    out_xfer->timeout_ms = 100;

    // 4. Disparar envío
    usb_host_transfer_submit(out_xfer);
}


void flush_buffer() {
    if (tx_len > 0) {
        esp_now_send(peerAddress, tx_buffer, tx_len);
        tx_len = 0; 
    }
}

// -------------------------------------------------------------------------
// 2. DRIVER CH340 (Extraído de usbothtest1.ino)
// -------------------------------------------------------------------------

void ch340_ctrl(uint8_t req, uint16_t val, uint16_t idx) {
    if (!ch340_dev_hdl || !ctrl_xfer) return;
    ctrl_xfer->device_handle = ch340_dev_hdl;
    ctrl_xfer->bEndpointAddress = 0; // [cite: 11]
    ctrl_xfer->callback = [](usb_transfer_t *t){ ctrl_done = true; };
    ctrl_xfer->num_bytes = 8; 
    usb_setup_packet_t *s = (usb_setup_packet_t *)(ctrl_xfer->data_buffer);
    // Definición exacta de tu archivo [cite: 12]
    s->bmRequestType = 0x40; s->bRequest = req; s->wValue = val; s->wIndex = idx; s->wLength = 0;
    ctrl_done = false;
    usb_host_transfer_submit_control(client_hdl, ctrl_xfer);
    while(!ctrl_done) { 
        uint32_t f; usb_host_lib_handle_events(1, &f); 
        usb_host_client_handle_events(client_hdl, 1); // [cite: 13]
    }
}

void init_ch340() {
    // Secuencia de inicialización exacta de tu archivo [cite: 14, 15]
    ch340_ctrl(0xA4, 0x0101, 0); delay(500); 
    ch340_ctrl(0x9A, 0x1312, 0xB202);
    // 9600 bps según tu comentario, ajustar si el HW requiere 115200
    ch340_ctrl(0x9A, 0x2727, 0);      // Enable UART
    delay(500);
    Serial0.println("CH340 listo (Modo Bridge)."); // [cite: 16]
}

// -------------------------------------------------------------------------
// 3. RECEPCIÓN Y BUCLE PRINCIPAL
// -------------------------------------------------------------------------

void in_cb(usb_transfer_t *t) {
    if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
        // Lógica de Wireless Avionics Bus: Llenado de buffer elástico
        for (int i = 0; i < t->actual_num_bytes; i++) {
            if (tx_len >= MAX_PAYLOAD) {
                flush_buffer();
            }
            tx_buffer[tx_len++] = t->data_buffer[i];
        }
        last_byte_time = millis();
    }
    // Re-submit para seguir escuchando [cite: 6]
    if (ch340_dev_hdl) usb_host_transfer_submit(t);
}

void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        Serial.printf("[DEBUG HOST] Dispositivo USB detectado en dirección: %d\n", event_msg->new_dev.address);
        if (usb_host_device_open(client_hdl, event_msg->new_dev.address, &ch340_dev_hdl) == ESP_OK) {
            usb_host_interface_claim(client_hdl, ch340_dev_hdl, 0, 0);
            init_ch340(); // [cite: 17]
            
            in_xfer->device_handle = ch340_dev_hdl;
            in_xfer->bEndpointAddress = 0x82; // Endpoint IN
            in_xfer->num_bytes = 32; // [cite: 17] Mantengo tu valor original de 32 bytes
            in_xfer->callback = in_cb;
            usb_host_transfer_submit(in_xfer);
        }
    }
}

void setup() {
  Serial0.begin(115200, SERIAL_8N1, RXD0, TXD0);

  // === OPTIMIZACIÓN LATENCIA (Stages 2, 3, 4) ===
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  // Eliminamos protocolo 11B para evitar negociación a 1Mbps
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_protocol(WIFI_IF_AP,  WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  // Fijar canal
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // Desactivar power save
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial0.println("Error ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent); // Ya no dará error con la nueva firma
  esp_now_register_recv_cb(OnDataRecv); // Ya no dará error con la nueva firma

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;  
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA; 

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial0.println("Error Peer");
    return;
  }

  // === USB HOST SETUP (Referencia usbothtest1.ino) ===
  static usb_host_config_t h_cfg = { .intr_flags = ESP_INTR_FLAG_LEVEL1 }; // [cite: 19]
  usb_host_install(&h_cfg);
  
  static usb_host_client_config_t c_cfg = { 
      .is_synchronous = false, 
      .max_num_event_msg = 10,
      .async = { .client_event_callback = client_event_cb, .callback_arg = NULL } 
  }; // [cite: 20]
  
  usb_host_client_register(&c_cfg, &client_hdl); // [cite: 21]
  Serial.println("[DEBUG HOST] Driver USB Host registrado. Esperando inserción física...");

  usb_host_transfer_alloc(8, 0, &ctrl_xfer); 
  usb_host_transfer_alloc(64, 0, &in_xfer); 
  usb_host_transfer_alloc(64, 0, &out_xfer); // Buffer para escribir al CH340
  
  Serial0.println("Sistema Host Iniciado.");
}

void loop() {
  uint32_t f; usb_host_lib_handle_events(1, &f); // [cite: 21]
  if (client_hdl) usb_host_client_handle_events(client_hdl, 1);

  // Flush dinámico del buffer para baja latencia
  if (tx_len > 0 && (millis() - last_byte_time > FLUSH_TIMEOUT_MS)) {
      flush_buffer();
  }
}