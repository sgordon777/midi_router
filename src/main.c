

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "ws2812.pio.h"

#include "hardware/uart.h"
#include "hardware/irq.h"


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


#define DELAY_START_FOR_CDC 0
#define WS_LATCH_DELAY 0
//#define PICO_OG
#define PICO_ZERO
#define VER 3





//--------------------------------------------------------------------+
// UART
//--------------------------------------------------------------------+

// ---- UART/MIDI config ----
#define MIDI_UART            uart0
#if defined(PICO_OG)
#define MIDI_UART_TX_PIN     16    // GPIO16 = TX
#define MIDI_UART_RX_PIN     17    // GPIO17 = RX
#elif defined (PICO_ZERO)
#define MIDI_UART_TX_PIN     12    
#define MIDI_UART_RX_PIN     13    
#endif

#define MIDI_UART_BAUD       31250 // MIDI standard
#define UART_RX_BUF_SIZE     256   // must be power of 2


static volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

// ---- Simple ring buffer for RX ----
static volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

static inline bool _rx_push(uint8_t b) {
    uint16_t next = (uart_rx_head + 1) & (UART_RX_BUF_SIZE - 1);
    if (next == uart_rx_tail) return false; // overflow -> drop
    uart_rx_buf[uart_rx_head] = b;
    uart_rx_head = next;
    return true;
}

static inline int _rx_pop(uint8_t *b) {
    if (uart_rx_head == uart_rx_tail) return -1; // empty
    *b = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) & (UART_RX_BUF_SIZE - 1);
    return 0;
}

// ---- RX IRQ: collect incoming UART bytes ----
static void __isr __time_critical_func(uart0_rx_irq_handler)(void) {
    while (uart_is_readable(MIDI_UART)) {
        uint8_t ch = uart_getc(MIDI_UART);
        (void)_rx_push(ch);
    }
}

// ---- Public: init UART0 on GPIO16/17 for MIDI ----
static inline void midi_uart_init(void) {
    // Pins to UART function
    gpio_set_function(MIDI_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MIDI_UART_RX_PIN, GPIO_FUNC_UART);

    // Core UART settings
    uart_init(MIDI_UART, MIDI_UART_BAUD);
    uart_set_hw_flow(MIDI_UART, false, false);
    uart_set_format(MIDI_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(MIDI_UART, true);

    // Clear any pending RX
    while (uart_is_readable(MIDI_UART)) (void)uart_getc(MIDI_UART);

    // Hook RX interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart0_rx_irq_handler);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(MIDI_UART, true, false); // RX IRQ on, TX IRQ off
}

// ---- Public: send ONE MIDI byte out UART ----
static inline void midi_uart_tx_byte(uint8_t b) {
    // block briefly if TX FIFO full
    while (!uart_is_writable(MIDI_UART)) { tight_loop_contents(); }
    uart_putc_raw(MIDI_UART, b);
}

// ---- Public: send a buffer (convenient for bursts) ----
static inline void midi_uart_write(const uint8_t* data, size_t len) {
    uart_write_blocking(MIDI_UART, data, len);
}

// ---- Optional: try-read ONE byte from UART RX buffer ----
static inline int midi_uart_try_read(uint8_t *out) {
    return _rx_pop(out); // 0 = got byte, -1 = none
}

// how many bytes are waiting?
static inline uint16_t uart_rx_available(void) {
    return (uart_rx_head - uart_rx_tail) & (UART_RX_BUF_SIZE - 1);
}

// pop one byte; returns 0 if got one, -1 if empty
static inline int uart_rx_pop(uint8_t *out) {
    if (uart_rx_head == uart_rx_tail) return -1;          // empty
    *out = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) & (UART_RX_BUF_SIZE - 1);
    return 0;
}


//--------------------------------------------------------------------+
// RGB
//--------------------------------------------------------------------+

#if defined(PICO_OG)
#define WS2812_PIN     12        // change if you wired a different GPIO
#elif defined (PICO_ZERO)
#define WS2812_PIN     16        // change if you wired a different GPIO
#endif

#define WS2812_IS_RGBW false    // set true if your LED is RGBW
#define WS2812_FREQ    800000   // WS2812 is ~800 kHz

static PIO  ws_pio = pio0;
static uint ws_sm  = 0;

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;


#include "hardware/pio.h"
#include "hardware/clocks.h"

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


//--------------------------------------------------------------------+
// MIDI
//--------------------------------------------------------------------+

typedef enum {
  STATE_WAITFOR_STATUS,
  STATE_WAITFOR_P1,
  STATE_WAITFOR_P2
} midi_state_t;

typedef struct midid_state_data_t {
  midi_state_t state;  // current state
  uint8_t status;    // status byte for current message
  uint8_t p1;        // parameter 1 for current message
  uint8_t cmd_len;   // len of message (1,2,3)
} midi_state_data_t;



// prototypes
uint8_t midi1_command_len(uint8_t cmd, uint8_t chn);
void midi_state_machine (uint8_t new_byte, midi_state_data_t* midi_state, uint8_t cable_num );
static inline void hsv_to_rgb_255(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);
void led_toggle(void);
void midi_task(void);



// main
int main(void) {
  board_init();

  
  //tusb_init(BOARD_TUD_RHPORT, &dev_init);
  tusb_init();


  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  // UART 
  midi_uart_init();

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

    // check for bytes form UART, and process them one by one
    uint8_t b;
    if (uart_rx_available()) {
        (void)uart_rx_pop(&b);
        midi_state_machine(b, &some_uart_state, 0);
}    

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

      // rout USB MIDI to UART MIDI
      // Note: this is a very simple implementation that does not handle SysEx
      // or running status. It just translates 1:1 USB MIDI packets to UART MIDI
      uint8_t cmdlen = midi1_command_len((packet[1]>>4)&0x0f, packet[1]&0x0f);
      midi_uart_write(packet, cmdlen);


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


void midi_state_machine (uint8_t new_byte, midi_state_data_t* midi_state, uint8_t cable_num )
{
  if (midi_state->state == STATE_WAITFOR_STATUS) 
  {
    if ( (new_byte & 0x80) == 0)
    {
      // highbit=0, we're in a "running state", continue as if in P1
      if (midi_state->cmd_len == 2)
      {
        // create a 2 byte MIDI packet and send to MIDI out
        uint8_t midi_1p0_msg[2] = { midi_state->status,  new_byte };
        tud_midi_stream_write(cable_num, midi_1p0_msg, 2);
        midi_state->state = STATE_WAITFOR_STATUS;
      }
      else
      {
        // wait for P2
        midi_state->p1 = new_byte;
        midi_state->state = STATE_WAITFOR_P2;
      }
    }
    else // new_byte & 0x80
    {
      // highbit=1, new status byte
      midi_state->status = new_byte;
      uint8_t cmd = midi_state->status >> 4;
      uint8_t chn = midi_state->status & 0x0f;
      if (cmd < 8)
      {
        // invalid CMD, error but stay in WAITFOR_STATUS
        // printf("invalid status byte!\n");
        return;
      }
      midi_state->cmd_len = midi1_command_len(cmd, chn); 
      if(midi_state->cmd_len == 1)
      {
        // create a 1 byte MIDI packet and send to MIDI out
        uint8_t midi_1p0_msg[1] = { midi_state->status };
        tud_midi_stream_write(cable_num, midi_1p0_msg, 1);
        midi_state->state = STATE_WAITFOR_STATUS;
      }
      else
      {
        // wait for P1
        midi_state->state = STATE_WAITFOR_P1;
      }
    } // new_byte & 0x80
  }
  else if (midi_state->state == STATE_WAITFOR_P1)
  {
    midi_state->p1 = new_byte;
    if (midi_state->cmd_len == 2)
    {
      // create a 2 byte MIDI packet and send to MIDI out
      uint8_t midi_1p0_msg[2] = { midi_state->status, midi_state->p1 };
      tud_midi_stream_write(cable_num, midi_1p0_msg, 2);
      midi_state->state = STATE_WAITFOR_STATUS;
    }
    else
    {
      midi_state->state = STATE_WAITFOR_P2;
    }
  }
  else if (midi_state->state == STATE_WAITFOR_P2)
  {
    // create a 3 byte MIDI packet and send to MIDI out
    uint8_t midi_1p0_msg[3] = { midi_state->status, midi_state->p1, new_byte };
    tud_midi_stream_write(cable_num, midi_1p0_msg, 3);
    midi_state->state = STATE_WAITFOR_STATUS;
  }

}


uint8_t midi1_command_len(uint8_t cmd, uint8_t chn) 
{
  // return len of MIDI command (1,2,3), or 0 if invalid
  // cmd is high nibble of status byte
  // chn is low nibble of status byte
  switch (cmd) {
    case 0x8: // Note Off
    case 0x9: // Note On
    case 0xA: // Polyphonic Key Pressure (Aftertouch)
    case 0xB: // Control Change
    case 0xE: // Pitch Bend Change
      return 3;
    case 0xC: // Program Change
    case 0xD: // Channel Pressure (After-touch)
      return 2;
    case 0xF: // System Common / Real Time Messages
      switch (chn) {
        case 0x0: // SysEx Start or continuation
        case 0x7: // SysEx End with following single byte
          return 0; // special handling needed, not supported here
        case 0x1: // MTC Quarter Frame
        case 0x3: // Song Select
          return 2;
        case 0x2: // Song Position Pointer
          return 3;
        case 0x6: // Tune Request
        case 0x8: // Timing Clock
        case 0xA: // Start
        case 0xB: // Continue
        case 0xC: // Stop
        case 0xE: // Active Sensing
        case 0xF: // System Reset
          return 1;
        default:
          return 0; // undefined system message, error but stay in WAITFOR_STATUS
      }
    default:
      return 0; // undefined message, error but stay in WAITFOR_STATUS
  }
}