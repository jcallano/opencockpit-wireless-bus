# Node B (Arduino CLI)

This peripheral is built and flashed with Arduino CLI to match ESP32 Arduino Core 3.3.5 behavior.

## Prereqs

- Install Arduino CLI.
- Install the ESP32 core 3.3.5:

```
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.3.5
```

## Build

```
arduino-cli compile --fqbn esp32:esp32:esp32s3 .
```

## Upload

```
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 .
```

## Monitor

```
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

If you need a different port, replace `/dev/ttyACM0` accordingly.

## Makefile helpers

```
make compile
make upload PORT=/dev/ttyACM0
make monitor PORT=/dev/ttyACM0 BAUD=115200
```
