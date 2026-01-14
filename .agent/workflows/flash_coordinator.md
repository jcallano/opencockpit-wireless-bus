---
description: Auto-Flash Coordinator Firmware (COM3)
---
This workflow automatically compiles and uploads the Coordinator firmware to COM3 without requiring confirmation for every step.

// turbo-all
1. Compile and Flash Coordinator
   Run: & "tools\bin\arduino-cli.exe" compile --fqbn esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=cdc,UploadMode=default firmware/coordinator/coordinator.ino --build-property "build.extra_flags=-DBOARD_ESPRESSIF_USB_OTG -DARDUINO_USB_MODE=1" --upload --port COM3
