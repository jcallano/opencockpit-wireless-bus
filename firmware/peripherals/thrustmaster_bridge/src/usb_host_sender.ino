#include <Arduino.h>
#include "usb/usb_host.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================
//  CONFIGURACIÓN
// =============================================================
// COMENTA ESTA LÍNEA PARA MODO VUELO (Rápido)
// DESCOMENTA PARA DEBUG (Ver Hex en Monitor Serie)
// #define DEBUG_USB_INPUT 

#define RX_PIN 44
#define TX_PIN 43
#define WIFI_CHANNEL 1

#define TM_VID 0x044F
#define PID_STICK    0x0405 
#define PID_QUADRANT 0x0407 

static const uint8_t PEER_MAC[6] = { 0xDC, 0xDA, 0x0C, 0x5A, 0x83, 0x64 }; 

// =============================================================
// 1. ESTRUCTURAS RAW (FINAL VALIDATED)
// =============================================================
#pragma pack(push, 1)

typedef struct {
    uint8_t reportId;       
    uint16_t x_axis;        // 14-bit
    uint16_t y_axis;        // 14-bit
    uint8_t z_twist;        // 8-bit
    uint8_t slider;         // 8-bit
    uint8_t hat_sw;         // Hat (4-bit low nibble)
    uint8_t buttons_a;      
    uint8_t buttons_b;      
} TCA_Stick_Raw_t;

typedef struct {
    uint8_t reportId;       // 0x01
    uint8_t b01_masters;    // Btns 17-24 (Masters, A/THR)
    uint8_t b02_detents;    // Btns 25-32 (Virtual Detents)
    uint16_t throttle_1;    
    uint16_t throttle_2;    
    uint16_t flaps;         
    uint8_t  b09_pk_rud;    // Btns 33-40 (Park Brk, Rud Trim)
    uint16_t speedbrake;    
    uint8_t  b12_gear;      // Btns 41-48 (Gear, AutoBrk, Spd Retracted)
    uint8_t  b13_spd_det;   // Btns 49-56 (Spd 1/2, 3/4, Full)
    uint8_t  b14_reserved;  // Btns 57-64
} TCA_Quadrant_Raw_t;

struct EsnStatePkt_v1_t {
  uint32_t ms;          
  uint16_t seq;
  uint8_t  magic;       
  uint8_t  version;     
  uint16_t axes_q12[8]; 
  uint8_t  buttons[8];  
  uint8_t  hat;         
  uint8_t  present;     
  uint8_t  padding[2];  
};
#pragma pack(pop)

struct AxisF_t {
  uint16_t filt_q12;
  uint16_t last_sent_q12;
  bool     init;
};

// =============================================================
// VARIABLES GLOBALES
// =============================================================
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_stick_buf[32]; 
static bool g_have_stick = false;
static uint8_t g_quad_buf[32];   
static bool g_have_quad = false;

static AxisF_t g_axes[8];
static uint16_t g_seq = 0;
static uint32_t g_last_sent_ms = 0;
static QueueHandle_t g_q = nullptr;

static const uint16_t AXIS_DEADBAND_Q12 = 3;  
static const uint32_t KEEPALIVE_MS = 50; 

#define MAX_DEVICES 2
struct Device_Info_t {
  bool active;
  usb_device_handle_t handle;
  uint16_t pid;
  uint8_t last_buf[64];
  uint16_t last_len;
};
static Device_Info_t devices[MAX_DEVICES];
static usb_host_client_handle_t client_hdl = NULL;
static usb_transfer_t *xfer_dev0 = NULL;
static usb_transfer_t *xfer_dev1 = NULL;
static usb_transfer_t *ctrl_xfer = NULL;

// =============================================================
// LÓGICA DE NEGOCIO
// =============================================================
void debug_hex_dump(const char* name, uint8_t* data, int len) {
  #ifdef DEBUG_USB_INPUT
  Serial.printf("[%s] ", name);
  for(int i=0; i<len; i++) Serial.printf("%02X ", data[i]);
  Serial.println();
  #endif
}

static inline bool AX_Update(AxisF_t *F, uint16_t new_q12) {
  if (!F->init) {
    F->init = true;
    F->filt_q12 = new_q12;
    F->last_sent_q12 = new_q12;
    return true; 
  }
  int32_t diff = (int32_t)new_q12 - (int32_t)F->filt_q12;
  F->filt_q12 = F->filt_q12 + (diff / 2); 

  int32_t d = (int32_t)F->filt_q12 - (int32_t)F->last_sent_q12;
  if (abs(d) >= AXIS_DEADBAND_Q12) {
    F->last_sent_q12 = F->filt_q12;
    return true;
  }
  return false;
}

// =============================================================
// LÓGICA PRINCIPAL: MAPEO Y DETECCIÓN DE CAMBIOS
// =============================================================
static void process_and_enqueue() {
  uint32_t now_ms = millis();
  
  // Variables estáticas para recordar el ÚLTIMO estado enviado
  // Esto es crucial para detectar cambios en botones instantáneamente
  static uint8_t last_sent_buttons[8] = {0};
  static uint8_t last_sent_hat = 0;

  TCA_Stick_Raw_t stick_local;
  TCA_Quadrant_Raw_t quad_local;
  bool haveS, haveQ;

  // Limpieza y Copia Atómica
  memset(&stick_local, 0, sizeof(stick_local));
  memset(&quad_local, 0, sizeof(quad_local));

  portENTER_CRITICAL(&g_lock);
  if (g_have_stick) memcpy(&stick_local, g_stick_buf, sizeof(TCA_Stick_Raw_t));
  if (g_have_quad)  memcpy(&quad_local,  g_quad_buf,  sizeof(TCA_Quadrant_Raw_t));
  haveS = g_have_stick;
  haveQ = g_have_quad;
  portEXIT_CRITICAL(&g_lock);

  EsnStatePkt_v1_t pkt;
  memset(&pkt, 0, sizeof(pkt)); 

  uint16_t raw_q12[8] = {0}; 

  // -----------------------------------------------------------
  // 1. PROCESAMIENTO STICK
  // -----------------------------------------------------------
  if (haveS) {
    raw_q12[0] = stick_local.x_axis >> 2;
    raw_q12[1] = stick_local.y_axis >> 2;
    raw_q12[2] = (uint16_t)stick_local.z_twist << 4;
    raw_q12[3] = (uint16_t)stick_local.slider << 4; 

    pkt.buttons[0] = stick_local.buttons_a;
    pkt.buttons[1] = stick_local.buttons_b;
    pkt.hat = stick_local.hat_sw & 0x0F; 
  }

  // -----------------------------------------------------------
  // 2. PROCESAMIENTO QUADRANT
  // -----------------------------------------------------------
  if (haveQ) {
    raw_q12[4] = (quad_local.throttle_1 + 8) >> 4;
    raw_q12[5] = (quad_local.throttle_2 + 8) >> 4;
    raw_q12[6] = (quad_local.flaps + 8) >> 4;
    raw_q12[7] = (quad_local.speedbrake + 8) >> 4;

    pkt.buttons[2] = quad_local.b01_masters;
    pkt.buttons[3] = quad_local.b02_detents;
    pkt.buttons[4] = quad_local.b09_pk_rud;
    
    // Byte 12: Gear / Autobrake / Spd Retracted
    // IMPORTANTE: Este es el que fallaba al ciclar. 
    // Ahora, cualquier cambio aquí disparará el envío.
    pkt.buttons[5] = quad_local.b12_gear; 
    
    pkt.buttons[6] = quad_local.b13_spd_det;
    pkt.buttons[7] = quad_local.b14_reserved; 
  }

  // -----------------------------------------------------------
  // 3. LÓGICA DE DISPARO (TRIGGER LOGIC)
  // -----------------------------------------------------------
  
  // A. Detectar cambios en EJES
  bool axis_changed = false;
  for(int i=0; i<8; i++) {
     // AX_Update devuelve true si el eje se movió más allá de la banda muerta
     axis_changed |= AX_Update(&g_axes[i], raw_q12[i]);
     pkt.axes_q12[i] = g_axes[i].last_sent_q12;
  }

  // B. Detectar cambios en BOTONES (NUEVO)
  // Comparamos la memoria del array de botones actual vs el último enviado
  bool btn_changed = (memcmp(pkt.buttons, last_sent_buttons, 8) != 0);
  
  // C. Detectar cambios en HAT
  bool hat_changed = (pkt.hat != last_sent_hat);

  // D. Keepalive (Timeout)
  bool keepalive = ((now_ms - g_last_sent_ms) >= KEEPALIVE_MS);
  
  // -----------------------------------------------------------
  // 4. ENVÍO
  // -----------------------------------------------------------
  // Enviamos si: Ejes cambiaron O Botones cambiaron O Hat cambió O pasó el tiempo
  if (axis_changed || btn_changed || hat_changed || keepalive) {
    
    pkt.magic = 0xA5;
    pkt.version = 1;
    pkt.seq = ++g_seq;
    pkt.ms = now_ms;
    pkt.present = (haveS ? 0x01 : 0) | (haveQ ? 0x02 : 0);
    
    xQueueOverwrite(g_q, &pkt);
    g_last_sent_ms = now_ms;

    // Actualizamos el estado "Last Sent" para la próxima comparación
    memcpy(last_sent_buttons, pkt.buttons, 8);
    last_sent_hat = pkt.hat;
  }
}

// =============================================================
// USB & SETUP
// =============================================================
static void usb_xfer_cb(usb_transfer_t *t) {
  if (t && t->device_handle && t->status == USB_TRANSFER_STATUS_COMPLETED) {
    int slot = -1;
    for(int i=0; i<MAX_DEVICES; i++) {
      if(devices[i].active && devices[i].handle == t->device_handle) { slot=i; break; }
    }

    if (slot >= 0 && t->actual_num_bytes > 0) {
      Device_Info_t *D = &devices[slot];
      bool changed = (D->last_len != t->actual_num_bytes) || 
                     (memcmp(D->last_buf, t->data_buffer, t->actual_num_bytes) != 0);

      if (changed) {
        memcpy(D->last_buf, t->data_buffer, t->actual_num_bytes);
        D->last_len = t->actual_num_bytes;

        if (D->pid == PID_STICK) {
          debug_hex_dump("STICK", t->data_buffer, t->actual_num_bytes);
          portENTER_CRITICAL(&g_lock);
          size_t len = (t->actual_num_bytes > sizeof(TCA_Stick_Raw_t)) ? sizeof(TCA_Stick_Raw_t) : t->actual_num_bytes;
          memcpy(g_stick_buf, t->data_buffer, len);
          g_have_stick = true;
          portEXIT_CRITICAL(&g_lock);
          process_and_enqueue();
        } 
        else if (D->pid == PID_QUADRANT) {
          debug_hex_dump("QUAD", t->data_buffer, t->actual_num_bytes);
          portENTER_CRITICAL(&g_lock);
          size_t len = (t->actual_num_bytes > sizeof(TCA_Quadrant_Raw_t)) ? sizeof(TCA_Quadrant_Raw_t) : t->actual_num_bytes;
          memcpy(g_quad_buf, t->data_buffer, len);
          g_have_quad = true;
          portEXIT_CRITICAL(&g_lock);
          process_and_enqueue();
        }
      }
    }
    usb_host_transfer_submit(t);
  }
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
  if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    usb_device_handle_t h;
    if (usb_host_device_open(client_hdl, event_msg->new_dev.address, &h) != ESP_OK) return;
    
    const usb_device_desc_t *dd;
    usb_host_get_device_descriptor(h, &dd);
    
    if (dd->idVendor != TM_VID) {
       usb_host_device_close(client_hdl, h);
       return;
    }

    int slot = -1;
    for(int i=0; i<MAX_DEVICES; i++) if(!devices[i].active) { slot=i; break; }
    if (slot < 0) { usb_host_device_close(client_hdl, h); return; }

    devices[slot].active = true;
    devices[slot].handle = h;
    devices[slot].pid = dd->idProduct;
    devices[slot].last_len = 0;

    Serial.printf("USB Device PID: %04X (Slot %d)\n", dd->idProduct, slot);
    usb_host_interface_claim(client_hdl, h, 0, 0); 

    usb_transfer_t *xfer = (slot==0) ? xfer_dev0 : xfer_dev1;
    xfer->device_handle = h;
    xfer->bEndpointAddress = 0x81; 
    xfer->num_bytes = 64;
    xfer->callback = usb_xfer_cb;
    usb_host_transfer_submit(xfer);
  }
  else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    for(int i=0; i<MAX_DEVICES; i++) {
        if (devices[i].active && devices[i].handle == event_msg->dev_gone.dev_hdl) {
            devices[i].active = false;
        }
    }
  }
}

static void usb_task(void *arg) {
  while (1) {
    usb_host_lib_handle_events(0, NULL);
    usb_host_client_handle_events(client_hdl, 0);
    vTaskDelay(1); 
  }
}

static void espnow_task(void *arg) {
  EsnStatePkt_v1_t pkt;
  while (1) {
    if (xQueueReceive(g_q, &pkt, portMAX_DELAY) == pdTRUE) {
      esp_now_send(PEER_MAC, (const uint8_t *)&pkt, sizeof(pkt));
    }
  }
}

static void espnow_send_cb(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);
  Serial.println("\n=== SENDER: AIRBUS TCA READY ===");

  g_q = xQueueCreate(1, sizeof(EsnStatePkt_v1_t));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(espnow_send_cb);
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, PEER_MAC, 6);
    p.channel = WIFI_CHANNEL;
    p.encrypt = false;
    esp_now_add_peer(&p);
  }

  const usb_host_config_t h_cfg = { .intr_flags = ESP_INTR_FLAG_LEVEL1 };
  usb_host_install(&h_cfg);
  
  static usb_host_client_config_t c_cfg = { 
    .is_synchronous = false, 
    .max_num_event_msg = 10,
    .async = { .client_event_callback = client_event_cb, .callback_arg = NULL } 
  };
  usb_host_client_register(&c_cfg, &client_hdl);

  usb_host_transfer_alloc(64, 0, &xfer_dev0);
  usb_host_transfer_alloc(64, 0, &xfer_dev1);
  usb_host_transfer_alloc(64, 0, &ctrl_xfer);

  xTaskCreatePinnedToCore(usb_task, "usb_task", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(espnow_task, "espnow_task", 4096, NULL, 5, NULL, 1);
}

void loop() { vTaskDelay(1000); }