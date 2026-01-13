/*
  Thrustmaster TCA Airbus HID Dump (ESP32-S3 USB Host)

  Hardware:
    - WeAct ESP32-S3 Dev Board Rev A (USB Host)
  Toolchain:
    - Arduino IDE
    - ESP32 Arduino Core v3.3.5 (ESP-IDF 5.5 based)
  Notes:
    - Tweaked with GPT-5.2 and Gemini 3 (debug/iterations)
    - Auto-discovers HID interface + Interrupt IN endpoint (no hardcoded 0x81)
    - Avoids forcing SET_CONFIGURATION unless needed (fallback only)
    - Prints decoded reports only when changed

  Tested devices:
    - Thrustmaster VID 0x044F
    - TCA Sidestick PID 0x0405
    - TCA Quadrant PID 0x0407
*/

#include <Arduino.h>
#include "usb/usb_host.h"

// ================================
// Optional ESP-NOW keepalive (disabled by default)
// ================================
#define ENABLE_ESPNOW 0

#if ENABLE_ESPNOW
  #include <WiFi.h>
  #include <esp_now.h>
  static uint8_t peerAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
#endif

// ================================
// HW (WeAct S3)
// ================================
#define RXD0 44
#define TXD0 43

// Thrustmaster
#define TM_VID 0x044F
#define PID_STICK    0x0405
#define PID_QUADRANT 0x0407

// ================================
// RAW structs (validated layout)
// ================================
#pragma pack(push, 1)
typedef struct {
  uint8_t  reportId;
  uint16_t x_axis;
  uint16_t y_axis;
  uint8_t  z_twist;
  uint8_t  slider;
  uint8_t  hat_sw;
  uint8_t  buttons_a;
  uint8_t  buttons_b;
} TCA_Stick_Data_t;

typedef struct {
  uint8_t  reportId;
  uint8_t  btn_byte_01; // B01
  uint8_t  btn_byte_02; // B02 (virtual detents + reverse flags)
  uint16_t throttle_1;  // T1
  uint16_t throttle_2;  // T2
  uint16_t flaps;       // FLP
  uint8_t  btn_byte_09; // B09
  uint16_t speedbrake;  // SPD
  uint8_t  btn_byte_12; // B12
  uint8_t  btn_byte_13; // B13
  uint8_t  btn_byte_14; // B14
} TCA_Quadrant_Data_t;
#pragma pack(pop)

// ================================
// Helpers
// ================================
static void printBin(uint8_t b) {
  for (int i = 7; i >= 0; i--) Serial0.print((b >> i) & 1);
}

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
} usb_desc_hdr_t;

// ================================
// USB Host globals
// ================================
#define MAX_DEVICES 2
#define XFER_BUF_SZ 64

typedef struct {
  bool active = false;

  usb_device_handle_t handle = NULL;
  uint16_t pid = 0;

  int      if_num = -1;
  uint8_t  ep_in  = 0;
  uint16_t mps    = 0;

  uint8_t  last_buf[XFER_BUF_SZ];
  uint16_t last_len = 0;
} Device_Info_t;

static Device_Info_t devices[MAX_DEVICES];

static usb_host_client_handle_t client_hdl = NULL;
static usb_transfer_t *xfer_dev0 = NULL;
static usb_transfer_t *xfer_dev1 = NULL;
static usb_transfer_t *ctrl_xfer = NULL;

// Control transfer tracking (prevents assert if close occurs while ctrl xfer is in-flight)
static volatile bool g_ctrl_inflight = false;
static usb_device_handle_t g_ctrl_dev = NULL;
static TaskHandle_t g_wait_task = nullptr;

// ================================
// CONTROL transfer sync wait
// ================================
static void ctrl_done_cb(usb_transfer_t *t) {
  g_ctrl_inflight = false;
  g_ctrl_dev = NULL;
  if (g_wait_task) xTaskNotifyGive(g_wait_task);
}

// SET_CONFIGURATION(1) sync (fallback only)
static bool set_config_1_sync(usb_device_handle_t handle, uint32_t timeout_ms = 1200) {
  if (!ctrl_xfer || !handle) return false;

  usb_setup_packet_t *setup = (usb_setup_packet_t *)ctrl_xfer->data_buffer;
  setup->bmRequestType = 0x00;
  setup->bRequest      = 0x09;      // SET_CONFIGURATION
  setup->wValue        = 1;         // configuration #1
  setup->wIndex        = 0;
  setup->wLength       = 0;

  ctrl_xfer->device_handle    = handle;
  ctrl_xfer->bEndpointAddress = 0;
  ctrl_xfer->num_bytes        = sizeof(usb_setup_packet_t); // MUST be 8
  ctrl_xfer->callback         = ctrl_done_cb;

  g_wait_task = xTaskGetCurrentTaskHandle();
  g_ctrl_inflight = true;
  g_ctrl_dev = handle;

  esp_err_t err = usb_host_transfer_submit_control(client_hdl, ctrl_xfer);
  if (err != ESP_OK) {
    g_ctrl_inflight = false;
    g_ctrl_dev = NULL;
    g_wait_task = nullptr;
    return false;
  }

  uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
  g_wait_task = nullptr;

  if (got == 0) {
    // Timeout: control transfer may still be in-flight
    Serial0.println("WARN: SET_CONFIGURATION timeout (no callback)");
    return false;
  }

  Serial0.printf("SET_CONFIGURATION done, xfer status=%d\n", (int)ctrl_xfer->status);
  return (ctrl_xfer->status == USB_TRANSFER_STATUS_COMPLETED);
}

// Find HID interface (class 0x03) alt=0 and an Interrupt IN endpoint from ACTIVE config
static bool find_hid_int_in_ep_active(usb_device_handle_t dev,
                                      int *out_if_num,
                                      uint8_t *out_ep_addr,
                                      uint16_t *out_mps)
{
  const usb_config_desc_t *cfg = nullptr;
  esp_err_t err = usb_host_get_active_config_descriptor(dev, &cfg);
  if (err != ESP_OK || !cfg) return false;

  const uint8_t *p = (const uint8_t *)cfg;
  const int total = cfg->wTotalLength;

  int cur_if = -1;
  int cur_alt = 0;
  uint8_t cur_class = 0;

  for (int off = 0; off + 2 <= total; ) {
    const usb_desc_hdr_t *h = (const usb_desc_hdr_t *)(p + off);
    if (h->bLength == 0) break;

    if (h->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      const usb_intf_desc_t *ifd = (const usb_intf_desc_t *)(p + off);
      cur_if    = ifd->bInterfaceNumber;
      cur_alt   = ifd->bAlternateSetting;
      cur_class = ifd->bInterfaceClass;
    }
    else if (h->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      const usb_ep_desc_t *ep = (const usb_ep_desc_t *)(p + off);

      if (cur_if >= 0 && cur_alt == 0 && cur_class == USB_CLASS_HID) {
        const bool is_in  = (ep->bEndpointAddress & 0x80) != 0;
        const bool is_int = (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT;

        if (is_in && is_int) {
          *out_if_num  = cur_if;
          *out_ep_addr = ep->bEndpointAddress;
          *out_mps     = ep->wMaxPacketSize;
          return true;
        }
      }
    }

    off += h->bLength;
  }

  return false;
}

// ================================
// Slot helpers
// ================================
static int find_slot_by_handle(usb_device_handle_t h) {
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (devices[i].active && devices[i].handle == h) return i;
  }
  return -1;
}

static void close_slot(int slot) {
  if (slot < 0 || slot >= MAX_DEVICES) return;
  if (!devices[slot].active) return;

  usb_device_handle_t h = devices[slot].handle;
  uint16_t pid = devices[slot].pid;

  Serial0.printf("Closing slot %d (pid:%04X hdl=%p)\n", slot, pid, h);

  // Prevent assert: do not close if a control transfer is still in-flight for this device
  if (g_ctrl_inflight && g_ctrl_dev == h) {
    Serial0.println("WARN: Not closing now (control transfer still in-flight). Marking inactive.");
    devices[slot].active = false;
    devices[slot].handle = NULL;
    return;
  }

  if (h) usb_host_device_close(client_hdl, h);
  devices[slot] = Device_Info_t{};
}

// ================================
// TRANSFER CALLBACK
// ================================
static void common_cb(usb_transfer_t *t) {
  if (!t || !t->device_handle) return;

  if (t->status != USB_TRANSFER_STATUS_COMPLETED) {
    usb_host_transfer_submit(t);
    return;
  }

  const int slot = find_slot_by_handle(t->device_handle);
  if (slot < 0) {
    usb_host_transfer_submit(t);
    return;
  }

  Device_Info_t &D = devices[slot];
  const uint16_t n = (uint16_t)t->actual_num_bytes;

  if (n > 0) {
    const bool changed =
      (D.last_len != n) ||
      (memcmp(D.last_buf, t->data_buffer, n) != 0);

    if (changed) {
      D.last_len = n;
      memcpy(D.last_buf, t->data_buffer, n);

      if (D.pid == PID_STICK && n >= sizeof(TCA_Stick_Data_t)) {
        const TCA_Stick_Data_t *s = (const TCA_Stick_Data_t *)t->data_buffer;

        Serial0.print("[STICK] ");
        Serial0.printf("IF:%d EP:0x%02X MPS:%u | ",
                       D.if_num, D.ep_in, (unsigned)D.mps);

        Serial0.printf("X:%05u Y:%05u Twist:%03u Slide:%03u | ",
                       (unsigned)s->x_axis, (unsigned)s->y_axis,
                       (unsigned)s->z_twist, (unsigned)s->slider);

        Serial0.printf("HAT:%X | BTN_A:", s->hat_sw);
        printBin(s->buttons_a);
        Serial0.print(" BTN_B:");
        printBin(s->buttons_b);
        Serial0.println();
      }
      else if (D.pid == PID_QUADRANT && n >= sizeof(TCA_Quadrant_Data_t)) {
        const TCA_Quadrant_Data_t *q = (const TCA_Quadrant_Data_t *)t->data_buffer;

        Serial0.print("[QUAD]  ");
        Serial0.printf("IF:%d EP:0x%02X MPS:%u | ",
                       D.if_num, D.ep_in, (unsigned)D.mps);

        Serial0.printf("T1:%05u T2:%05u FLP:%05u SPD:%05u",
                       (unsigned)q->throttle_1, (unsigned)q->throttle_2,
                       (unsigned)q->flaps, (unsigned)q->speedbrake);

        Serial0.print("\n        ");
        Serial0.print("B01:"); printBin(q->btn_byte_01);
        Serial0.print(" B02:"); printBin(q->btn_byte_02);
        Serial0.print(" B09:"); printBin(q->btn_byte_09);
        Serial0.print(" B12:"); printBin(q->btn_byte_12);
        Serial0.print(" B13:"); printBin(q->btn_byte_13);
        Serial0.print(" B14:"); printBin(q->btn_byte_14);
        Serial0.println("\n");
      }
      else {
        // Unknown/short report -> minimal hexdump (first 32 bytes)
        Serial0.printf("[PID:%04X] IF:%d EP:0x%02X len:%u data:",
                       D.pid, D.if_num, D.ep_in, (unsigned)n);
        for (int i = 0; i < n && i < 32; i++) {
          Serial0.printf(" %02X", ((uint8_t*)t->data_buffer)[i]);
        }
        if (n > 32) Serial0.print(" ...");
        Serial0.println();
      }

#if ENABLE_ESPNOW
      // Optional keepalive packet
      uint8_t two_bytes[2] = { (D.pid == PID_STICK) ? 1 : 2, 0 };
      esp_now_send(peerAddress, two_bytes, sizeof(two_bytes));
#endif
    }
  }

  usb_host_transfer_submit(t);
}

// ================================
// USB EVENT CALLBACK
// ================================
static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
  if (!event_msg) return;

  if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    const uint8_t addr = event_msg->new_dev.address;

    usb_device_handle_t h = NULL;
    if (usb_host_device_open(client_hdl, addr, &h) != ESP_OK || !h) return;

    const usb_device_desc_t *dd = nullptr;
    usb_host_get_device_descriptor(h, &dd);

    if (!dd || dd->idVendor != TM_VID) {
      usb_host_device_close(client_hdl, h);
      return;
    }

    int slot = -1;
    for (int i = 0; i < MAX_DEVICES; i++) {
      if (!devices[i].active) { slot = i; break; }
    }
    if (slot < 0) {
      Serial0.println("No free slots");
      usb_host_device_close(client_hdl, h);
      return;
    }

    devices[slot] = Device_Info_t{};
    devices[slot].active = true;
    devices[slot].handle = h;
    devices[slot].pid    = dd->idProduct;

    Serial0.printf(">>> NEW DEV addr:%u VID:%04X PID:%04X slot:%d <<<\n",
                   addr, dd->idVendor, dd->idProduct, slot);

    // 1) Try active config first (no SET_CONFIGURATION)
    int if_num = -1;
    uint8_t ep_in = 0;
    uint16_t mps = 0;

    bool ok_desc = find_hid_int_in_ep_active(h, &if_num, &ep_in, &mps);

    // 2) Fallback: SET_CONFIGURATION(1) only if descriptor not available
    if (!ok_desc) {
      Serial0.println("Active config not ready / HID EP not found. Trying SET_CONFIGURATION(1) fallback...");
      (void)set_config_1_sync(h, 1200);
      ok_desc = find_hid_int_in_ep_active(h, &if_num, &ep_in, &mps);
    }

    if (!ok_desc) {
      Serial0.println("ERROR: No HID Interrupt IN endpoint found.");
      close_slot(slot);
      return;
    }

    if (mps == 0) mps = XFER_BUF_SZ;
    if (mps > XFER_BUF_SZ) mps = XFER_BUF_SZ;

    devices[slot].if_num = if_num;
    devices[slot].ep_in  = ep_in;
    devices[slot].mps    = mps;

    Serial0.printf("HID: IF=%d EP=0x%02X MPS=%u\n", if_num, ep_in, (unsigned)mps);

    // 3) Claim interface
    esp_err_t cl = usb_host_interface_claim(client_hdl, h, if_num, 0);
    if (cl != ESP_OK) {
      Serial0.printf("interface_claim failed: %d\n", (int)cl);
      close_slot(slot);
      return;
    }

    // 4) Submit IN transfer
    usb_transfer_t *xfer = (slot == 0) ? xfer_dev0 : xfer_dev1;
    xfer->device_handle    = h;
    xfer->bEndpointAddress = ep_in;
    xfer->num_bytes        = mps;
    xfer->callback         = common_cb;

    usb_host_transfer_submit(xfer);
  }
  else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    // In ESP-IDF 5.x: DEV_GONE provides dev_hdl
    usb_device_handle_t gone = event_msg->dev_gone.dev_hdl;
    Serial0.printf("<<< DEV GONE hdl=%p <<<\n", gone);

    int slot = find_slot_by_handle(gone);
    if (slot >= 0) close_slot(slot);
  }
}

// ================================
// SETUP / LOOP
// ================================
void setup() {
  Serial0.begin(115200, SERIAL_8N1, RXD0, TXD0);
  delay(600);
  Serial0.println("\n=== Thrustmaster HID dump (auto IF/EP/MPS) ===");

#if ENABLE_ESPNOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, peerAddress, 6);
    p.channel = 1;
    p.encrypt = false;
    esp_now_add_peer(&p);
  }
#endif

  // USB Host install
  const usb_host_config_t h_cfg = {
    .intr_flags = ESP_INTR_FLAG_LEVEL1
  };
  usb_host_install(&h_cfg);

  static usb_host_client_config_t c_cfg = {
    .is_synchronous = false,
    .max_num_event_msg = 10,
    .async = {
      .client_event_callback = client_event_cb,
      .callback_arg = NULL
    }
  };
  usb_host_client_register(&c_cfg, &client_hdl);

  // Allocate transfers
  usb_host_transfer_alloc(XFER_BUF_SZ, 0, &xfer_dev0);
  usb_host_transfer_alloc(XFER_BUF_SZ, 0, &xfer_dev1);
  usb_host_transfer_alloc(XFER_BUF_SZ, 0, &ctrl_xfer);
}

void loop() {
  uint32_t flags = 0;
  usb_host_lib_handle_events(1, &flags);
  if (client_hdl) usb_host_client_handle_events(client_hdl, 1);
}

