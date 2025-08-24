

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "ws2812.pio.h"

// ADDED: Pico SDK headers for stdio and timing helpers
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
/* This MIDI example send sequence of note (on/off) repeatedly. To test on PC, you need to install
 * synth software and midi connection management software. On
 * - Linux (Ubuntu): install qsynth, qjackctl. Then connect TinyUSB output port to FLUID Synth input port
 * - Windows: install MIDI-OX
 * - MacOS: SimpleSynth
 */

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static inline void hsv_to_rgb_255(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);


#define WS2812_PIN     16        // change if you wired a different GPIO
#define WS2812_IS_RGBW false    // set true if your LED is RGBW
#define WS2812_FREQ    800000   // WS2812 is ~800 kHz
#define DELAY_START_FOR_CDC 0
#define WS_LATCH_DELAY 0

static PIO  ws_pio = pio0;
static uint ws_sm  = 0;

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_toggle(void);
void midi_task(void);

//static void ws2812_init(void);
//static inline void ws_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#include "hardware/pio.h"
#include "hardware/clocks.h"

//--------------------------------------------------------------------+
// RGB
//--------------------------------------------------------------------+

static inline uint32_t pack_grb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}
static inline void ws_put(uint32_t grb) {
  // WS2812 program expects 24 bits left-justified (<< 8)
  pio_sm_put_blocking(ws_pio, ws_sm, grb << 8u);
}
static void ws2812_init(void) {
  uint offset = pio_add_program(ws_pio, &ws2812_program);
  ws2812_program_init(ws_pio, ws_sm, offset, WS2812_PIN, WS2812_FREQ, WS2812_IS_RGBW);
}
static inline void ws_set_rgb(uint8_t r, uint8_t g, uint8_t b) { ws_put(pack_grb(r,g,b)); }




/*------------- MAIN -------------*/
int main(void) {
  board_init();


  
  // init device stack on configured roothub port
  
  /*
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  */
  
  
  //tusb_init(BOARD_TUD_RHPORT, &dev_init);
  tusb_init();


  if (board_init_after_tusb) {
    board_init_after_tusb();
  }
  // Initialize stdio backends enabled in CMake (USB CDC in your case)
  stdio_init_all();


/*
  This loop waits until the serial-usb is ready so any startup messages can be seen.
  if startup messages aren't important, then set DELAY_START_FOR_CDC to 0
*/
#if DELAY_START_FOR_CDC
  absolute_time_t until = make_timeout_time_ms(2000);
  while (!stdio_usb_connected() && absolute_time_diff_us(get_absolute_time(), until) > 0) {
    tud_task();
    sleep_ms(10);
  }
#endif

  ws2812_init();               // <-- add this
  ws_set_rgb(0, 255, 0);         // dim red at boot (optional)
  sleep_us(80);  // WS2812 latch/reset (>50 µs)


  fflush(stdout); // optional
  printf("hello from pi pico!\n");
  led_toggle();
  while (1) {
    tud_task(); // tinyusb device task
    midi_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}



inline uint8_t SATU8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
const uint8_t note_sequence[] = {
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};

void midi_task(void)
{
  static uint32_t start_ms = 0;

  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  while (tud_midi_available()) 
  {
    uint8_t packet[4];
    static uint8_t v = 255;
    static uint8_t s = 0;
    tud_midi_packet_read(packet);
    led_toggle();
    printf("recd MIDI packet: 0x%.2x,0x%.2x,0x%.2x,0x%.2x\n", packet[0], packet[1], packet[2], packet[3]);
    switch (packet[1] & 0xf0) {
      case 0x80: // Note Off
        s = 0xff;
        break;
      case 0x90: // Note On
        s = 0xff;
        break;
      default:
        s = 0x80;
        break;
    }
    uint8_t h = SATU8(packet[2] * 2); // note
    uint8_t vtest = SATU8(packet[3]); // velocity
    if (vtest != 0) {
      v = vtest;
    }
    uint8_t r, g, b;
    hsv_to_rgb_255(h, s, v, &r, &g, &b);
    ws_set_rgb(r, g, b);
#if WS_LATCH_DELAY
    sleep_us(80);  // WS2812 latch/reset (>50 µs)
#endif

  }

  // send note periodically
  if (board_millis() - start_ms < 286) {
    return; // not enough time
  }
  start_ms += 286;

  // Previous positions in the note sequence.
  int previous = (int) (note_pos - 1);

  // If we currently are at position 0, set the
  // previous position to the last note in the sequence.
  if (previous < 0) {
    previous = sizeof(note_sequence) - 1;
  }

  // Send Note On for current position at full velocity (127) on channel 1.
  uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
  tud_midi_stream_write(cable_num, note_on, 3);

  // Send Note Off for previous note.
  uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
  tud_midi_stream_write(cable_num, note_off, 3);

  // Increment position
  note_pos++;

  // If we are at the end of the sequence, start over.
  if (note_pos >= sizeof(note_sequence)) {
    note_pos = 0;
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_toggle(void)
{
  static int led_state = 1;
  board_led_write(led_state);
  led_state = 1 - led_state; // toggle

}



static inline void hsv_to_rgb_255(uint8_t h, uint8_t s, uint8_t v,
                                  uint8_t *r, uint8_t *g, uint8_t *b) {
    if (s == 0) { *r = *g = *b = v; return; }    // gray

    uint16_t region    = h / 43;                  // 0..5
    uint16_t remainder = (h - region * 43) * 6;   // 0..258

    uint16_t p = (uint16_t)v * (255 - s) / 255;
    uint16_t q = (uint16_t)v * (255 - (s * remainder) / 255) / 255;
    uint16_t t = (uint16_t)v * (255 - (s * (255 - remainder)) / 255) / 255;

    switch (region) {
      case 0: *r = v; *g = t; *b = p; break;
      case 1: *r = q; *g = v; *b = p; break;
      case 2: *r = p; *g = v; *b = t; break;
      case 3: *r = p; *g = q; *b = v; break;
      case 4: *r = t; *g = p; *b = v; break;
      default:*r = v; *g = p; *b = q; break;      // region 5
    }
}