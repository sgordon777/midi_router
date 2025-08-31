#include "bsp/board_api.h"
#include "tusb.h"

/* PID auto-mapping (make interface set unique per VID/PID) */
#define _PID_MAP(itf, n)  ((CFG_TUD_##itf) << (n))
#define USB_PID (0x4000 | _PID_MAP(CDC,0) | _PID_MAP(MSC,1) | _PID_MAP(HID,2) | \
                 _PID_MAP(MIDI,3) | _PID_MAP(VENDOR,4))

// ---------------- Device Descriptor ----------------
tusb_desc_device_t const desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor           = 0xCafe,
  .idProduct          = USB_PID,
  .bcdDevice          = 0x0100,

  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,

  .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
  return (uint8_t const*)&desc_device;
}

// ---------------- Configuration Descriptor ----------------
enum {
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_MIDI,
  ITF_NUM_MIDI_STREAMING,
  ITF_NUM_TOTAL
};

/* FIX 1: include CDC length */
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MIDI_DESC_LEN)

/* Endpoints:
   CDC:  notif IN 0x81,  data OUT 0x02, data IN 0x82
   MIDI: OUT  0x01,      IN  0x83  (avoid 0x81/0x82 to prevent conflicts)
*/
#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define EPNUM_MIDI_OUT    0x01
#define EPNUM_MIDI_IN     0x83  // FIX 2: unique from CDC endpoints

uint8_t const desc_fs_configuration[] = {
  // config num, itf count, string idx, total len, attribute, power (mA)
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // CDC (Comm + Data): itf, stridx,   ep notif, ep size, ep out,      ep in,     ep size
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 0,    EPNUM_CDC_NOTIF, 8,    EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

  // MIDI: itf, stridx, ep out,        ep in,        wMaxPacketSize
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 0,  EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 64),
};

#if TUD_OPT_HIGH_SPEED
uint8_t const desc_hs_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 0, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 512),
  TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 0, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 512),
};
#endif

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
#if TUD_OPT_HIGH_SPEED
  return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_hs_configuration : desc_fs_configuration;
#else
  return desc_fs_configuration;
#endif
}

// ---------------- String Descriptors ----------------
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

char const* string_desc_arr[] = {
  (const char[]){ 0x09, 0x04 }, // 0: English (0x0409)
  "pigo_corp",                    // 1: Manufacturer
  "usb_midi_router",             // 2: Product
  NULL,                         // 3: Serial = board unique ID
};

static uint16_t _desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  size_t chr_count;

  switch (index) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = board_usb_get_serial(_desc_str + 1, 32);
      break;

    default: {
      if (!(index < (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))) return NULL;
      const char* str = string_desc_arr[index];
      chr_count = strlen(str);
      size_t max_count = (sizeof(_desc_str) / sizeof(_desc_str[0])) - 1;
      if (chr_count > max_count) chr_count = max_count;
      for (size_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
    } break;
  }

  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}