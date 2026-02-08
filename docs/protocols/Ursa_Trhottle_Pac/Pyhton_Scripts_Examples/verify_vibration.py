
import hid
import time
import sys

# VID/PID for WinWing Ursa Minor 32 Throttle
VID = 0x4098
PID = 0xB920

def send_vibe(device, control_id, intensity):
    buffer = [0] * 64
    buffer[0] = 0x02
    buffer[1] = 0x10
    buffer[2] = 0xb9
    buffer[3] = 0x00
    buffer[4] = 0x00
    buffer[5] = 0x03
    buffer[6] = 0x49
    buffer[7] = control_id
    buffer[8] = intensity
    
    try:
        device.write(buffer)
    except Exception as e:
        print(f"Error en ID 0x{control_id:02x}: {e}")

def main():
    try:
        device = hid.device()
        device.open(VID, PID)
    except Exception as e:
        print(f"Dispositivo no encontrado: {e}")
        return

    try:
        for motor_name, motor_id in [("Motor 1", 0x0E), ("Motor 2", 0x0F)]:
            print(f"--- Barrido de intensidad {motor_name} (ID 0x{motor_id:02x}) ---")
            # Subir progresivamente
            for i in [0, 64, 128, 192, 255]:
                print(f"Intensidad: {i}/255", end="\r")
                send_vibe(device, motor_id, i)
                time.sleep(0.4)
            # Bajar progresivamente
            for i in [192, 128, 64, 0]:
                print(f"Intensidad: {i}/255", end="\r")
                send_vibe(device, motor_id, i)
                time.sleep(0.4)
            print(f"\n{motor_name} detenido.")
        
        print("\nPrueba de intensidad completada y motores detenidos.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        device.close()

if __name__ == "__main__":
    main()
