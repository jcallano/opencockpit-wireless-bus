# Agent Context & Environment Configuration

This document provides essential context for AI agents working on this repository, specifically regarding local tool paths and environment configurations.

## 1. Local Tool Paths

**Do not assume tools are in the global system PATH.** Use the following relative paths from the repository root:

*   **Arduino CLI**: `tools\bin\arduino-cli.exe`
*   **Python Interpreter**: `.venv\Scripts\python.exe`
*   **Pip**: `.venv\Scripts\pip.exe`

## 2. Arduino Environment

The local Arduino CLI is configured with the following:

*   **Config File**: Default local configuration (check `tools/bin/arduino-cli.yaml` or default AppData location if generated there).
*   **ESP32 Core Version**: `3.3.5` (Platform: `esp32:esp32`)
*   **Additional URLs**: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`

### Common Commands
*   **Compile**: `tools\bin\arduino-cli.exe compile --fqbn esp32:esp32:esp32s3 firmware/coordinator`
*   **Upload**: `tools\bin\arduino-cli.exe upload -p COMx --fqbn esp32:esp32:esp32s3 firmware/coordinator`

## 3. Python Environment

A local virtual environment is set up in `.venv`.

### Activation
*   **Windows Powershell**: `.venv\Scripts\Activate.ps1`
*   **Command Line**: `.venv\Scripts\activate.bat`

### Dependencies
Dependencies are listed in `software/requirements.txt`.
To install/update:
```powershell
.venv\Scripts\pip.exe install -r software\requirements.txt
```

## 4. Repository Structure Highlights

*   `firmware/`: ESP32 firmware code (PlatformIO/Arduino compatible).
*   `software/`: PC side Python bridge software.
*   `tools/bin/`: Local binary tools (Arduino CLI).
