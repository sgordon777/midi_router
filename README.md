VER 0.7
-add note button
-fix USB->UART routing
-UART->USB latency: 80
-button->usb latency: 75


Composite USB device for RP2040 exposing **USB-MIDI** and **USB-CDC (stdio)**.
-Take MIDI input from USB
-Take MIDI input from UART
-Rout MIDI from USB to MIDI UART
-Rout MIDI from UART to MIDI USB
-decode keyscans, create packet and send to MIDI USB or MIDI UART (dip switch?)
-Print debug messages via pi's USB-serial

## Prereqs
- Pico SDK extension and all its dependencies

## Build
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build -j
OR
./buildall.ps1

## TODO
add key scanner for my keyboard
add midi state machine and router for routing UART MIDI->USB MIDI and MIDI-USB to UART USB



A few quick pro-tips now that you’ve got CDC + MIDI running:

Windows driver cache: If you change the interface mix again (e.g., add HID later), bump idProduct or bcdDevice so Windows doesn’t reuse the old driver binding.

Don’t lose first prints: Either keep the short wait loop you added or set
add_compile_definitions(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000) in CMake.

Avoid blocking when no terminal is open:
    
#define LOGF(...) do { if (stdio_usb_connected()) printf(__VA_ARGS__); } while(0)


WSL reminder: The COM port shows up in Windows, not inside WSL.

MIDI RX sanity: You’re already draining tud_midi_available(); that prevents DAWs from blocking on send.

Tools

# mpremote is a useful command-line terminal:
mpremote devs               # query ports
mpremote connect COM5       # connect
