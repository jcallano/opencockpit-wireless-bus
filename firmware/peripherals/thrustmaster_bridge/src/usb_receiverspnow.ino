#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "USB.h"
#include "USBHID.h"

// =============================================================
// CONFIGURACIÓN
// =============================================================
static const uint8_t ESPNOW_CHANNEL = 1;

// =============================================================
// ESTRUCTURA DE TRANSPORTE (IGUAL AL SENDER)
// =============================================================
#pragma pack(push, 1)
struct EsnStatePkt_v1_t {
  uint32_t ms;          
  uint16_t seq;
  uint8_t  magic;       
  uint8_t  version;     
  uint16_t axes_q12[8]; 
  uint8_t  buttons[8];  // 64 botones totales
  uint8_t  hat;         
  uint8_t  present;     
  uint8_t  padding[2];  
};
#pragma pack(pop)

// =============================================================
// ESTRUCTURAS DE REPORTE PARA WINDOWS (SPLIT)
// =============================================================
#pragma pack(push, 1)

// JOYSTICK 1: Ejes + Hat + 32 Botones (Stick + Motores)
struct Joy1_Report_t {
  uint16_t axes[8];     
  uint8_t  hat; 
  uint32_t buttons;     // 32 botones (4 bytes)
};

// JOYSTICK 2: Solo 32 Botones (Sistemas: Gear, Brake, etc.)
struct Joy2_Report_t {
  uint32_t buttons;     // 32 botones (4 bytes)
};

#pragma pack(pop)

// =============================================================
// DESCRIPTOR HID MULTI-DISPOSITIVO
// =============================================================
static const uint8_t joy_report_descriptor[] = {
  // -------------------------------------------------
  // DISPOSITIVO 1: REPORT ID 1 (Ejes + Hat + Btn 1-32)
  // -------------------------------------------------
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        // REPORT ID 1

  // --- 8 EJES (16 bits) ---
  0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 
  0x09, 0x34, 0x09, 0x35, 0x09, 0x36, 0x09, 0x37,
  0x15, 0x00, 0x27, 0xFF, 0xFF, 0x00, 0x00, 
  0x75, 0x10, 0x95, 0x08, 0x81, 0x02,

  // --- HAT SWITCH ---
  0x05, 0x01, 0x09, 0x39, 
  0x15, 0x00, 0x25, 0x07, 
  0x35, 0x00, 0x46, 0x3E, 0x01, 
  0x65, 0x14, 
  0x75, 0x04, 0x95, 0x01, 0x81, 0x42, // Null state
  0x75, 0x04, 0x95, 0x01, 0x81, 0x03, // Padding

  // --- 32 BOTONES (Joystick 1) ---
  0x05, 0x09, 
  0x19, 0x01, 0x29, 0x20, // 1 a 32
  0x15, 0x00, 0x25, 0x01, 
  0x75, 0x01, 0x95, 0x20, // 32 count
  0x81, 0x02,

  0xC0, // End Collection ID 1

  // -------------------------------------------------
  // DISPOSITIVO 2: REPORT ID 2 (Btn 33-64)
  // -------------------------------------------------
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x05,        // Usage (Game Pad) - Para diferenciar icono si quieres
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x02,        // REPORT ID 2

  // --- 32 BOTONES (Joystick 2) ---
  // Representan los botones físicos 33-64 del sistema total
  0x05, 0x09, 
  0x19, 0x01, 0x29, 0x20, // 1 a 32 (Lógicos para este dispositivo)
  0x15, 0x00, 0x25, 0x01, 
  0x75, 0x01, 0x95, 0x20, 
  0x81, 0x02,

  0xC0  // End Collection ID 2
};

class USBHIDJoystick : public USBHIDDevice {
public:
  USBHIDJoystick(USBHID &hid) : _hid(hid) {
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      _hid.addDevice(this, sizeof(joy_report_descriptor));
    }
  }
  uint16_t _onGetDescriptor(uint8_t *dst) override {
    memcpy(dst, joy_report_descriptor, sizeof(joy_report_descriptor));
    return sizeof(joy_report_descriptor);
  }
  // Funciones separadas para enviar cada reporte
  void sendJoy1(const Joy1_Report_t &r) {
    _hid.SendReport(1, &r, sizeof(r));
  }
  void sendJoy2(const Joy2_Report_t &r) {
    _hid.SendReport(2, &r, sizeof(r));
  }
private:
  USBHID &_hid;
};

static USBHID hid;
static USBHIDJoystick joy(hid);
static QueueHandle_t g_q = nullptr;

// =============================================================
// TAREA USB (DIVISOR DE DATOS)
// =============================================================
static void usb_hid_task(void *arg) {
  EsnStatePkt_v1_t pkt;
  Joy1_Report_t r1;
  Joy2_Report_t r2;
  
  while(1) {
    if (xQueueReceive(g_q, &pkt, portMAX_DELAY) == pdTRUE) {
      if (pkt.magic != 0xA5) continue;

      // --- PREPARAR JOYSTICK 1 ---
      memset(&r1, 0, sizeof(r1));
      // Ejes
      for(int i=0; i<8; i++) r1.axes[i] = pkt.axes_q12[i] << 4;
      // Hat
      r1.hat = pkt.hat & 0x0F;
      // Botones 1-32 (Bytes 0-3 del array original)
      memcpy(&r1.buttons, &pkt.buttons[0], 4);

      // --- PREPARAR JOYSTICK 2 ---
      memset(&r2, 0, sizeof(r2));
      // Botones 33-64 (Bytes 4-7 del array original)
      memcpy(&r2.buttons, &pkt.buttons[4], 4);
      
      // Filtro Byte 14 del Quadrant (El último byte de r2)
      // pkt.buttons[7] corresponde al byte más alto de r2.buttons
      // Lo limpiamos usando máscara binaria: 0x00FFFFFF (borra el byte alto)
      // O simplemente modificamos el byte raw en el struct si es más fácil:
      ((uint8_t*)&r2.buttons)[3] = 0; 

      // --- ENVIAR A WINDOWS ---
      joy.sendJoy1(r1);
      joy.sendJoy2(r2);
    }
  }
}

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(EsnStatePkt_v1_t)) return;
  xQueueOverwriteFromISR(g_q, data, NULL);
}

void setup() {
  // Cambio de identidad por seguridad (opcional pero recomendado)
  USB.PID(0xBBBB); 
  USB.productName("Airbus Dual Controller");
  USB.manufacturerName("SimProject");
  USB.begin();
  hid.begin();

  Serial.begin(115200);
  Serial.println("Receiver Dual-Device Mode");

  g_q = xQueueCreate(1, sizeof(EsnStatePkt_v1_t));
  
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  if (esp_now_init() != ESP_OK) while(1);
  esp_now_register_recv_cb((esp_now_recv_cb_t)on_recv);

  xTaskCreatePinnedToCore(usb_hid_task, "usb_hid_task", 4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelay(1000); }

