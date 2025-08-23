// tusb_config.h  — TinyUSB configuration for RP2040 (Pico)
// Place this in your project root.

#pragma once

//--------------------------------------------------------------------+
// Common
//--------------------------------------------------------------------+

// TinyUSB debug level: 0 = off, 1 = errors, 2 = warnings, 3 = info
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

// MCU + OS (Pico SDK integrates TinyUSB with its own OS layer)
#define CFG_TUSB_MCU          OPT_MCU_RP2040
#define CFG_TUSB_OS           OPT_OS_PICO

// Use TinyUSB device stack at Full Speed on RHPORT0
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// Control endpoint size (FS = 64)
#define CFG_TUD_ENDPOINT0_SIZE 64

// Memory alignment for USB DMA (safe default for RP2040)
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN    __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------+
// Device Class Configuration
//--------------------------------------------------------------------+

// Enable only the classes you use
#define CFG_TUD_CDC           1   // needed for USB-serial / pico_stdio_usb
#define CFG_TUD_MIDI          1   // USB MIDI device

// (Explicitly disable others just to be clear)
#define CFG_TUD_HID           0
#define CFG_TUD_MSC           0
#define CFG_TUD_VENDOR        0
#define CFG_TUD_NET           0
#define CFG_TUD_BTH           0
#define CFG_TUD_AUDIO         0
#define CFG_TUD_VIDEO         0

//----------------------- CDC (USB-Serial) ----------------------------

// Endpoint packet size at FS is 64
#define CFG_TUD_CDC_EP_BUFSIZE   64

// Driver ring buffers (host->device RX, device->host TX)
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256

//----------------------- MIDI ---------------------------------------

// Endpoint packet size at FS is 64
#define CFG_TUD_MIDI_EP_BUFSIZE  64

// Driver ring buffers for MIDI streaming
#define CFG_TUD_MIDI_RX_BUFSIZE  128
#define CFG_TUD_MIDI_TX_BUFSIZE  128