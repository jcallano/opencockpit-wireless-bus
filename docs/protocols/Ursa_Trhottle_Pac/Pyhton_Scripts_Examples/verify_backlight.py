
import hid
import time

# VID/PID for WinWing Ursa Minor 32 Throttle
VID = 0x4098
PID = 0xB920

def set_intensity(device, cmd_type, control_id, value):
    buffer = [0] * 64
    buffer[0] = 0x02
    buffer[1] = cmd_type # Importante: 0x10 o 0x01
    buffer[2] = 0xb9
    buffer[3] = 0x00
    buffer[4] = 0x00
    buffer[5] = 0x03
    buffer[6] = 0x49
    buffer[7] = control_id
    buffer[8] = value
    
    try:
        device.write(buffer)
    except Exception as e:
        print(f"Error en Cmd {cmd_type:02x} ID {control_id:02x}: {e}")

def main():
    try:
        device = hid.device()
        device.open(VID, PID)
    except Exception as e:
        print(f"Dispositivo no encontrado: {e}")
        return
    
    try:
        zones = [
            ("1. Throttle Backlight", 0x10, 0x00),
            ("2. Marker Light", 0x10, 0x02),
            ("3. Flaps/Spoiler Backlight", 0x01, 0x00),
            ("4. Digital Tube (LCD) Backlight", 0x01, 0x02)
        ]

        for name, cmd, cid in zones:
            print(f"Probando: {name} (Cmd 0x{cmd:02x}, ID 0x{cid:02x})")
            for lap in range(2):
                # Subida rápida
                for i in [0, 128, 255]:
                    set_intensity(device, cmd, cid, i)
                    time.sleep(0.3)
                # Bajada rápida
                for i in [128, 0]:
                    set_intensity(device, cmd, cid, i)
                    time.sleep(0.3)
            
            print(f"  {name} completado.")
            time.sleep(0.5)

        print("\nPrueba de Backlight (4 Zonas) finalizada.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        device.close()

if __name__ == "__main__":
    main()
