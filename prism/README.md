# Prism — USB Serial LED Controller

ESP-IDF firmware for the Seeed XIAO ESP32-C3. Receives RGB pixel data from the
RoboRIO over USB CDC serial and drives up to 4 WS281x LED strips via the RMT
peripheral.

## Build

```bash
cd prism
pio run                  # compile
pio run -t upload        # flash via USB
pio device monitor       # serial monitor at 2 Mbaud
```

## Pin Assignments

| Strip | GPIO |
|-------|------|
| 0     | 2    |
| 1     | 3    |
| 2     | 4    |
| 3     | 5    |
