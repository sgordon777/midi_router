

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
// Concifguration and settings
//--------------------------------------------------------------------+

// version
#define VER 7
// features
#define FEATURE_C_BUTTON
#define FEATURE_UART_TO_USB_MIDI_ROUTER
#define FEATURE_USB_TO_UART_MIDI_ROUTER
//define FEATURE_PLAY_TUNE_ON_USB_MIDI
// configuration
//#define MODULE_PICO_OG
#define MODULE_PICO_ZERO
#define DELAY_START_FOR_CDC 0
#define WS_LATCH_DELAY 0
#define DEBOUNCE_LIM 1000
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






//--------------------------------------------------------------------+
// GPIO
//--------------------------------------------------------------------+

#ifdef MODULE_PICO_ZERO
#define GPIO_IN_A 4 
#define GPIO_OUT_A 2  // change if you want others
#define GPIO_OUT_B 3
#endif

static inline int button_read(void) 
{
  static int ctr = DEBOUNCE_LIM;
  static int outval = 0;
  if (++ctr >= DEBOUNCE_LIM ) 
  { 
    outval = 1 - gpio_get(GPIO_IN_A); 
    ctr = 0;
  }
  return outval;
}


static inline void din_init_pullup(void) {
    gpio_init(GPIO_IN_A);
    gpio_set_dir(GPIO_IN_A, GPIO_IN);
    gpio_pull_up(GPIO_IN_A);           // input idles HIGH
}



static inline void two_gpio_init_outputs(void) {
    gpio_init(GPIO_OUT_A); gpio_set_dir(GPIO_OUT_A, GPIO_OUT); gpio_put(GPIO_OUT_A, 0);
    gpio_init(GPIO_OUT_B); gpio_set_dir(GPIO_OUT_B, GPIO_OUT); gpio_put(GPIO_OUT_B, 0);
}

static inline void gpio_a_set(bool v) { gpio_put(GPIO_OUT_A, v); }
static inline void gpio_b_set(bool v) { gpio_put(GPIO_OUT_B, v); }

// handy toggles
static inline void gpio_a_toggle(void) { gpio_xor_mask(1u << GPIO_OUT_A); }
static inline void gpio_b_toggle(void) { gpio_xor_mask(1u << GPIO_OUT_B); }


//--------------------------------------------------------------------+
// UART
//--------------------------------------------------------------------+

// ---- UART/MIDI config ----
#define MIDI_UART            uart0
#if defined(MODULE_PICO_OG)
#define MIDI_UART_TX_PIN     16    // GPIO16 = TX
#define MIDI_UART_RX_PIN     17    // GPIO17 = RX
#elif defined (MODULE_PICO_ZERO)
#define MIDI_UART_TX_PIN     0
#define MIDI_UART_RX_PIN     1
#endif

#define MIDI_UART_BAUD       31250 // MIDI standard
#define UART_RX_BUF_SIZE     256   // must be power of 2



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

#if defined(MODULE_PICO_OG)
#define WS2812_PIN     21        // change if you wired a different GPIO
#elif defined (MODULE_PICO_ZERO)
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
static inline void hsv_to_rgb_255(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);
void visualize_ws2812(uint8_t* midi_message);
void led_toggle(void);


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

uint8_t midi1_command_len(uint8_t cmd, uint8_t chn);
void uart_midi_in_state_machine (uint8_t new_byte, midi_state_data_t* midi_state, uint8_t cable_num );
void usb_midi_in_uart_midi_out(void);
void play_tune_task(void);


//--------------------------------------------------------------------+
// Main
//--------------------------------------------------------------------+



// main
int main(void) 
{
  midi_state_data_t state = {STATE_WAITFOR_STATUS, 0, 0, 0};
  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1
  int butval, old_butval=0;
  int keyval = 0, old_keyval=0;

  // board init
  board_init();

  // IO init
  two_gpio_init_outputs();
  din_init_pullup();

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
  gpio_a_set(1);
  gpio_b_set(1);
  while (1) 
  {
    tud_task(); // tinyusb device task
    usb_midi_in_uart_midi_out();
#ifdef FEATURE_C_BUTTON
    butval = button_read();
    int c = getchar_timeout_us(0);  // 0 = poll immediately
    if (c != PICO_ERROR_TIMEOUT) 
    {
       keyval = 1;
    }
    else
    {
       keyval = 0;
    }
    if ( (butval != old_butval) || (keyval!=old_keyval) )
    {
      // if button status changed, send a "middle C" note_on message to USB
      uint8_t note_on[3] = { 0x90 | channel, 0x3c, 77 * MAX(butval,keyval) }; // on if butval=1, off if butval=0
      tud_midi_stream_write(cable_num, note_on, 3); 
      printf("button event detected, sending noteOn: 0x%.2x,0x%.2x,0x%.2x\n", note_on[0], note_on[1], note_on[2]);
    }
    old_butval = butval;
    old_keyval = keyval;
#endif // FEATURE_C_BUTTON
#ifdef FEATURE_PLAY_TUNE_ON_USB_MIDI
    play_tune_task();
#endif // FEATURE_PLAY_TUNE_ON_USB_MIDI
    // check for inactivity and blink LED

#ifdef FEATURE_UART_TO_USB_MIDI_ROUTER
    // check for bytes form UART, and process them one by one
    uint8_t b;
    if (uart_rx_available()) 
    {
        (void)uart_rx_pop(&b);
        uart_midi_in_state_machine(b, &state, cable_num);
    }
#endif // FEATURE_UART_TO_USB_MIDI_ROUTER

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

void usb_midi_in_uart_midi_out(void)
{

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  while (tud_midi_available()) 
  {
    uint8_t packet[4];
    tud_midi_packet_read(packet);
    printf("recd MIDI message from USB: 0x%.2x,0x%.2x,0x%.2x,0x%.2x\n", packet[0], packet[1], packet[2], packet[3]);
#ifdef FEATURE_USB_TO_UART_MIDI_ROUTER    
    uint8_t cmdlen = midi1_command_len((packet[1]>>4)&0x0f, packet[1]&0x0f);
    midi_uart_write(&packet[1], cmdlen);
#endif // FEATURE_USB_TO_UART_MIDI_ROUTER
    visualize_ws2812(&packet[1]);
  }
#ifdef FEATURE_USB_TO_UART_MIDI_ROUTER

#endif // FEATURE_USB_TO_UART_MIDI_ROUTER
}

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
const uint8_t note_sequence[] = {
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};

void play_tune_task(void)
{

  static uint32_t start_ms = 0;

  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1


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


void uart_midi_in_state_machine (uint8_t new_byte, midi_state_data_t* midi_state, uint8_t cable_num )
{
  int output_len = 0;
  uint8_t b0=0, b1=0, b2=0; // max len=3

  if (midi_state->state == STATE_WAITFOR_STATUS) 
  {
    if ( (new_byte & 0x80) == 0)
    {
      // highbit=0, we're in a "running state", continue as if in P1
      if (midi_state->cmd_len == 2)
      {
        // create a 2 byte MIDI packet and send to MIDI out
        output_len = 2;
        b0=midi_state->status; b1=new_byte; b2=0;
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
        output_len = 1;
        b0=midi_state->status; b1=0; b2=0;
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
      output_len = 2;
      b0=midi_state->status; b1=midi_state->p1; b2=0; 
    }
    else
    {
      midi_state->state = STATE_WAITFOR_P2;
    }
  }
  else if (midi_state->state == STATE_WAITFOR_P2)
  {
    // create a 3 byte MIDI packet and send to MIDI out
    output_len = 3;
    b0=midi_state->status; b1=midi_state->p1; b2=new_byte;
  }

  // if we have a complete packet, send it out and go back to WAITFOR_STATUS
  if (output_len > 0)
  {
        uint8_t midi_1p0_msg[3] = { b0, b1, b2 }; // max len=3
        if (midi_1p0_msg[0] != 0xfe) // ignore active sensing
        {
          tud_midi_stream_write(cable_num, midi_1p0_msg, output_len);
          printf("recd MIDI message from UART[len=%d]: 0x%.2x,0x%.2x,0x%.2x\n", output_len, b0, b1, b2);
          visualize_ws2812(midi_1p0_msg);
        }

        gpio_a_toggle();
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

void visualize_ws2812(uint8_t* midi_msg)
{
    static uint8_t v = 255;
    static uint8_t s = 0;
    static uint8_t h = 0;
    uint8_t vtest = 0;

    uint8_t r, g, b;

    if (midi_msg[0] == 0xfe) return; // ignore active sensing
    switch (midi_msg[0] & 0xf0) 
    {
      case 0x80: // Note Off
      case 0x90: // Note On
        s = 0xff;
        h = SATU8(midi_msg[1] * 2); // note
        vtest = SATU8(midi_msg[2]); // velocity
        if (vtest != 0)    v = vtest;
        break;

      case 0xa0: // polyphony
        s = 0x80;
        vtest = SATU8(midi_msg[2]);
        if (vtest != 0)    v = vtest;
        break;

      case 0xb0: // control change
        s = 0x80;
        vtest = SATU8(midi_msg[2]);
        if (vtest != 0)    v = vtest;
        break;

      case 0xc0: // program change
        h = 0xf0;
        s = 0xff;
        vtest = SATU8(midi_msg[1]*16);
        if (vtest != 0)    v = vtest;
        break;

      case 0xd0: // pressure
        s = 0x20;
        vtest = SATU8(midi_msg[2]);
        if (vtest != 0)    v = vtest;
        break;

      case 0xe0: // bend
        s = 0xc0;
        vtest = SATU8(midi_msg[2]);
        if (vtest != 0)    v = vtest;
        break;


      case 0xF0: // SysEx
        s = 0xff;
        h = 0x00;
        v = 0xff;
        break;



      default:
        s = 0x80;
        break;
    }
    hsv_to_rgb_255(h, s, v, &r, &g, &b);
    ws_set_rgb(r, g, b);
#if WS_LATCH_DELAY
    sleep_us(80);  // WS2812 latch/reset (>50 µs)
#endif

  }


