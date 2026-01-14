#!/usr/bin/env bash
set -euo pipefail

root_dir="/home/jcallano/chatgpt2/chatgpt/opencockpit-wireless-bus"

for _ in {1..10}; do
  if [[ -d /dev/serial/by-id ]]; then
    break
  fi
  sleep 1
done
if [[ ! -d /dev/serial/by-id ]]; then
  echo "ERROR: /dev/serial/by-id not found" >&2
  exit 1
fi

node_a_cdc=$(readlink -f /dev/serial/by-id/usb-Espressif_Systems_ESP32S3_DEV_*-if00 2>/dev/null | head -n 1 || true)
if [[ -z "${node_a_cdc}" && -e /dev/ttyACM1 ]]; then
  node_a_cdc=/dev/ttyACM1
fi
node_c_uart=$(readlink -f /dev/serial/by-id/usb-1a86_USB_Single_Serial_* 2>/dev/null | head -n 1 || true)

if [[ -z "${node_a_cdc}" || ! -e "${node_a_cdc}" ]]; then
  echo "ERROR: Node A CDC not found" >&2
  exit 1
fi

if [[ -z "${node_c_uart}" || ! -e "${node_c_uart}" ]]; then
  echo "ERROR: Node C UART not found" >&2
  exit 1
fi

echo "Node A CDC:  ${node_a_cdc}"
echo "Node C UART: ${node_c_uart}"

python3 - <<PY
import time
import serial

def reset_port(port):
    ser = serial.Serial(port, 115200, timeout=0.2)
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    ser.dtr = False
    ser.close()

reset_port("${node_c_uart}")
PY

echo "Capturing Node C UART log (5s)..."
python3 - <<PY
import sys, time
import serial

port = "${node_c_uart}"
ser = serial.Serial(port, 115200, timeout=0.2)
end = time.time() + 5
while time.time() < end:
    data = ser.read(1024)
    if data:
        sys.stdout.buffer.write(data)
        sys.stdout.flush()
ser.close()
PY

node_a_cdc=""
for _ in {1..10}; do
  node_a_cdc=$(readlink -f /dev/serial/by-id/usb-Espressif_Systems_ESP32S3_DEV_*-if00 2>/dev/null | head -n 1 || true)
  if [[ -z "${node_a_cdc}" && -e /dev/ttyACM1 ]]; then
    node_a_cdc=/dev/ttyACM1
  fi
  if [[ -n "${node_a_cdc}" && -e "${node_a_cdc}" ]]; then
    break
  fi
  sleep 1
done
if [[ -z "${node_a_cdc}" || ! -e "${node_a_cdc}" ]]; then
  echo "ERROR: Node A CDC not found after reset" >&2
  exit 1
fi

echo "Running test_runner on ${node_a_cdc}..."
python3 "${root_dir}/software/tools/test_runner.py" \
  --port "${node_a_cdc}" \
  --node-id 0x02 \
  --expected-nodes 1 \
  --discovery-timeout 5.0 \
  --latency-samples 50 \
  --latency-timeout 0.2 \
  --throughput-count 500 \
  --throughput-timeout 10.0 \
  --payload-size 16
