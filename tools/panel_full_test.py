# ==============================================================
# Herramienta de Diagnostico Interactiva Rowsfire B107
# Desarrollada por: @jcallano
# Fecha: 13 de Junio de 2026 Ver 1.1 RAW MONITOR ADDED.
# Con mucho amor para la comunidad simmer
# ==============================================================

import serial
import time
import os
import sys

# Intentar importar msvcrt para lectura no bloqueante de teclado en Windows
try:
    import msvcrt
    msvcrt_available = True
except ImportError:
    msvcrt_available = False

# Configuracion por defecto
HW_PORT = 'COM3'
BAUD_RATE = 57600
CALIBRATION_FACTOR = 1.175

# Lista de LEDs y sus descripciones en espanol (sin acentos para evitar problemas de codificacion)
LED_IDS = [
    "FPA1LP", "FPF1LP", "FPCLLP", "FPCRLP", "FPF2LP", "FPA2LP", "MBLED", "LOP", 
    "APUFAULT", "APUOVERSPEED", "LSOFF", "LTBOFF", "RSOFF", "RTBOFF", 
    "D1D", "SPOLED", "D2D", "LGOB", "RGOB", "AGOB", "YDOL", 
    "HPAE1LP", "HPAE2LP", "HPBE1LP", "HPBE2LP", "HPAOH", "HPBOH", 
    "WAILVO", "WAIRVO", "EAILCAI", "EAIRCAI", "EAILCVO", "EAIRCVO", 
    "WHLSO", "WHLFO", "WHRFO", "WHRSO", "WHLSOH", "WHLFOH", "WHRFOH", "WHRSOH",
    "LDG_FLAP3_LAMP", "CREW_SUPPLY_SW", "PUMPS-L-TK-1-2"
]

LED_DESCRIPTIONS = {
    "FPA1LP": "Bomba de Combustible Aft 1 Presion Baja (Tanque Izquierdo)",
    "FPF1LP": "Bomba de Combustible Fwd 1 Presion Baja (Tanque Izquierdo)",
    "FPCLLP": "Bomba de Combustible Central Izquierda Presion Baja",
    "FPCRLP": "Bomba de Combustible Central Derecha Presion Baja",
    "FPF2LP": "Bomba de Combustible Fwd 2 Presion Baja (Tanque Derecho)",
    "FPA2LP": "Bomba de Combustible Aft 2 Presion Baja (Tanque Derecho)",
    "MBLED": "LED de Descarga de Bateria Principal (BAT DISCHARGE)",
    "LOP": "Presion Baja de Aceite de la APU (LOW OIL PRESSURE)",
    "APUFAULT": "Fallo de la APU (APU FAULT)",
    "APUOVERSPEED": "Sobrevelocidad de la APU (APU OVERSPEED)",
    "LSOFF": "Generador de Motor Izquierdo Off Bus (SOURCE OFF)",
    "LTBOFF": "Bus de Transferencia Izquierdo Desconectado (TRANSFER BUS OFF)",
    "RSOFF": "Generador de Motor Derecho Off Bus (SOURCE OFF)",
    "RTBOFF": "Bus de Transferencia Derecho Desconectado (TRANSFER BUS OFF)",
    "D1D": "Desconexion de Drive del Generador 1 (DRIVE 1 DISCONNECT)",
    "SPOLED": "Energia de Reserva Apagada (STANDBY POWER OFF)",
    "D2D": "Desconexion de Drive del Generador 2 (DRIVE 2 DISCONNECT)",
    "LGOB": "Generador Izquierdo Fuera de Bus (GEN OFF BUS)",
    "RGOB": "Generador Derecho Fuera de Bus (GEN OFF BUS)",
    "AGOB": "Generador de APU Fuera de Bus (APU GEN OFF BUS)",
    "YDOL": "Amortiguador de Guiada Apagado (YAW DAMPER OFF)",
    "HPAE1LP": "Bomba Hidraulica A de Motor 1 Presion Baja",
    "HPAE2LP": "Bomba Hidraulica A de Motor 2 Presion Baja",
    "HPBE1LP": "Bomba Hidraulica B de Motor 1 Presion Baja",
    "HPBE2LP": "Bomba Hidraulica B de Motor 2 Presion Baja",
    "HPAOH": "Sobretemperatura de Bomba Hidraulica A (OVERHEAT)",
    "HPBOH": "Sobretemperatura de Bomba Hidraulica B (OVERHEAT)",
    "WAILVO": "Valvula Izquierda de Anti-Ice de Ala Abierta (VALVE OPEN)",
    "WAIRVO": "Valvula Derecha de Anti-Ice de Ala Abierta (VALVE OPEN)",
    "EAILCAI": "Engine Anti-Ice Izquierdo COLD/ANTI-ICE",
    "EAIRCAI": "Engine Anti-Ice Derecho COLD/ANTI-ICE",
    "EAILCVO": "Valvula de Anti-Ice de Motor Izquierdo Abierta (VALVE OPEN)",
    "EAIRCVO": "Valvula de Anti-Ice de Motor Derecho Abierta (VALVE OPEN)",
    "WHLSO": "Calefaccion de Ventanilla Lateral Izquierda Encendida (WINDOW HEAT ON)",
    "WHLFO": "Calefaccion de Ventanilla Frontal Izquierda Encendida (WINDOW HEAT ON)",
    "WHRFO": "Calefaccion de Ventanilla Frontal Derecha Encendida (WINDOW HEAT ON)",
    "WHRSO": "Calefaccion de Ventanilla Lateral Derecha Encendida (WINDOW HEAT ON)",
    "WHLSOH": "Sobrecalentamiento de Ventanilla Lateral Izquierda (OVERHEAT)",
    "WHLFOH": "Sobrecalentamiento de Ventanilla Frontal Izquierda (OVERHEAT)",
    "WHRFOH": "Sobrecalentamiento de Ventanilla Frontal Derecha (OVERHEAT)",
    "WHRSOH": "Sobrecalentamiento de Ventanilla Lateral Derecha (OVERHEAT)",
    "LDG_FLAP3_LAMP": "Transito de Flaps de Borde de Ataque (LEADING EDGE FLAPS TRANSIT)",
    "CREW_SUPPLY_SW": "Alimentacion de Oxigeno de Tripulacion Encendida (CREW OXYGEN ON)",
    "PUMPS-L-TK-1-2": "Luz de Alerta Combinada de Bombas de Combustible"
}

# Mapa detallado de todos los switches e interruptores del Overhead B107
SWITCHES_SPEC = {
    "LUCES / MISC": {
        "LANDINGL": ["UP", "DOWN"],
        "LANDINGR": ["UP", "DOWN"],
        "RUNWAYL": ["UP", "DOWN"],
        "RUNWAYR": ["UP", "DOWN"],
        "TAXILT": ["UP", "DOWN"],
        "LOGOLT": ["UP", "DOWN"],
        "WINGLT": ["UP", "DOWN"],
        "WWELL": ["UP", "DOWN"],
        "ACLIGHTS": ["UP", "DOWN"],
        "PSTEADY": ["UP", "MID", "DOWN"],
        "PMODE": ["UP", "MID", "DOWN"],
        "DOMEWHITEDIM": ["UP", "MID", "DOWN"],
        "ACCAB": ["UP", "DOWN"],
        "ECSUPPLY": ["UP", "DOWN"],
        "ECEXHAUST": ["UP", "DOWN"],
        "CHIMEONLY": ["UP", "DOWN"]
    },
    "FUEL": {
        "FPFWDL": ["UP", "DOWN"],
        "FPAFTL": ["UP", "DOWN"],
        "FPCTRL": ["UP", "DOWN"],
        "FPCTRR": ["UP", "DOWN"],
        "FPFWDR": ["UP", "DOWN"],
        "FPAFTR": ["UP", "DOWN"],
        "FPCROSSFEED": ["UP", "DOWN"],
        "YAWDAMPER": ["UP", "DOWN"]
    },
    "ELEC / PWR": {
        "MASTERBAT": ["UP", "DOWN"],
        "STANDBYPOWER": ["UP", "MID", "DOWN"],
        "BUSTRANSFER": ["UP", "DOWN"],
        "EGEN1ON": ["PRESS", "RELEASE"],
        "EEN1OFF": ["PRESS", "RELEASE"],
        "EGEN2ON": ["PRESS", "RELEASE"],
        "EGEN2OFF": ["PRESS", "RELEASE"],
        "EAPUGEN1ON": ["PRESS", "RELEASE"],
        "EAPUGEN1OFF": ["PRESS", "RELEASE"],
        "EAPUGEN2ON": ["PRESS", "RELEASE"],
        "EAPUGEN2OFF": ["PRESS", "RELEASE"],
        "GNDPWRON": ["PRESS", "RELEASE"],
        "GNDPWROFF": ["PRESS", "RELEASE"]
    },
    "AIR / PNEU": {
        "BLEEDENG1": ["UP", "DOWN"],
        "BLEEDAPU": ["UP", "DOWN"],
        "BLEEDENG2": ["UP", "DOWN"],
        "BLEEDLPACK": ["UP", "MID", "DOWN"],
        "BLEEDRPACK": ["UP", "MID", "DOWN"],
        "BLEEDVALVE": ["UP", "MID", "DOWN"],
        "TRIMAIR": ["UP", "DOWN"],
        "BLRFAN": ["UP", "DOWN"],
        "BRRFAN": ["UP", "DOWN"]
    },
    "ANTI-ICE / WINDOW HEAT": {
        "PROBEA": ["UP", "DOWN"],
        "PROBEB": ["UP", "DOWN"],
        "WINGANTIICE": ["UP", "DOWN"],
        "ENG1ANTIICE": ["UP", "DOWN"],
        "ENG2ANTIICE": ["UP", "DOWN"],
        "WHLSIDE": ["UP", "DOWN"],
        "WHLFWD": ["UP", "DOWN"],
        "WHRFWD": ["UP", "DOWN"],
        "WHRSIDE": ["UP", "DOWN"]
    },
    "PROTECTION / CABIN": {
        "ATTEND": ["PRESS", "RELEASE"],
        "GRDCALL": ["PRESS", "RELEASE"],
        "OVHTTEST": ["PRESS", "RELEASE"],
        "TRIPRESET": ["PRESS", "RELEASE"],
        "GDDISCLEFT": ["PRESS", "RELEASE"],
        "GDDRIGHTGUARD": ["PRESS", "RELEASE"],
        "WHOVHTUP": ["PRESS", "RELEASE"],
        "WHOVHTDOWN": ["PRESS", "RELEASE"],
        "OVCLOSE": ["PRESS", "RELEASE"],
        "OVOPEN": ["PRESS", "RELEASE"],
        "EMEREXITLT": ["UP", "DOWN"],
        "FBELTS": ["UP", "DOWN"],
        "ESBOTHL": ["UP", "MID", "DOWN"]
    },
    "SELECTORES / ENC": {
        "ENGINESTART1": ["0", "1", "2", "3"],
        "ENGINESTART2": ["0", "1", "2", "3"],
        "APUSTART": ["PRESS", "RELEASE"],
        "APUSTARTOFF": ["UP", "DOWN"],
        "IRSL": ["0", "1", "2", "3"],
        "IRSR": ["0", "1", "2", "3"],
        "RWIPER": ["0", "1", "2", "3"],
        "VHFNAVBOTH": ["UP", "MID", "DOWN"],
        "IRSBOTH": ["UP", "MID", "DOWN"],
        "FMCBOTH": ["UP", "MID", "DOWN"],
        "ENCLANDALT": ["LEFT", "RIGHT"],
        "ENCFLTALT": ["LEFT", "RIGHT"]
    }
}

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def print_header():
    print("==============================================================")
    print("      Rowsfire B107 Ultimate Interactive Diagnostic Tool      ")
    print("--------------------------------------------------------------")
    print(" Desarrollado por: @jcallano")
    print(" Fecha: 13 de Junio de 2026")
    print(" Con mucho amor para la comunidad simmer (B737 Overhead)")
    print("==============================================================")

def parse_line(line):
    """Parsea una linea recibida del panel (puede ser snapshot separado por ';' o evento simple)"""
    results = []
    if not line:
        return results
    if "=" in line:
        parts = line.split(";")
        for p in parts:
            if "=" in p:
                try:
                    k, v = p.split("=")
                    results.append((k.strip(), v.strip()))
                except ValueError:
                    pass
    return results

# --- TEST 1: DISPLAYS DE ALTITUD ---
def test_displays(ser):
    while True:
        clear_screen()
        print_header()
        print("\n--- TEST DE DISPLAYS (FLT ALT / LAND ALT) ---")
        print("1. Barrido automatico (Secuencia 00000 -> 99999)")
        print("2. Entrada manual de valores personalizados")
        print("3. Apagar Displays (Mostrar '----')")
        print("0. Regresar al menu principal")
        print("--------------------------------------------------------------")
        choice = input("Selecciona una opcion: ").strip()

        if choice == "1":
            speed_in = input("Introduce velocidad de retardo por digito en segundos (por defecto 0.8s): ").strip()
            try:
                delay = float(speed_in) if speed_in else 0.8
            except ValueError:
                delay = 0.8

            print("\n[*] Iniciando barrido. Presiona Ctrl+C para cancelar...")
            try:
                for i in range(10):
                    val_flt = str(i) * 5
                    val_land = str(i) * 4
                    print(f"  [>] Enviando a Displays: FLT_ALT={val_flt} | LAND_ALT={val_land}")
                    ser.write(f"FLT_ALT={val_flt}\r\n".encode('ascii'))
                    ser.write(f"LAND_ALT={val_land}\r\n".encode('ascii'))
                    time.sleep(delay)
                print("[OK] Barrido completado.")
                time.sleep(1)
            except KeyboardInterrupt:
                print("\n[!] Barrido interrumpido.")
                time.sleep(1)

        elif choice == "2":
            flt_val = input("Introduce valor para FLT ALT (max 5 digitos, ej: 32000): ").strip()
            land_val = input("Introduce valor para LAND ALT (max 4 digitos, ej: 1450): ").strip()
            if flt_val:
                ser.write(f"FLT_ALT={flt_val}\r\n".encode('ascii'))
            if land_val:
                ser.write(f"LAND_ALT={land_val}\r\n".encode('ascii'))
            print("[*] Valores enviados a los displays.")
            time.sleep(1)

        elif choice == "3":
            print("[*] Apagando displays...")
            ser.write(b"FLT_ALT=----\r\n")
            ser.write(b"LAND_ALT=----\r\n")
            time.sleep(1)

        elif choice == "0":
            break

# --- TEST 2: AGUJA APU EGT ---
def test_egt(ser):
    while True:
        clear_screen()
        print_header()
        print("\n--- TEST DE AGUJA APU EGT ---")
        print("1. Barrido automatico de temperatura (0 C -> 1100 C -> 0 C)")
        print("2. Sincronizar Cero Fisico (Procedimiento STALL de tope)")
        print("3. Introducir grados manuales (C)")
        print("4. Apagar indicador (EGT=OFF)")
        print("0. Regresar al menu principal")
        print("--------------------------------------------------------------")
        choice = input("Selecciona una opcion: ").strip()

        if choice == "1":
            print("\n[*] Iniciando barrido calibrado...")
            points = [0, 200, 400, 600, 800, 1000, 1100, 0]
            for p in points:
                raw = int(p / CALIBRATION_FACTOR)
                print(f"  [>] Posicion: {p} C (Comando Raw enviado: EGT={raw})")
                ser.write(f"EGT={raw}\r\n".encode('ascii'))
                time.sleep(1.8)
            print("[OK] Barrido finalizado.")
            time.sleep(1)

        elif choice == "2":
            print("\n--- INICIANDO RESET DE CERO (STALL) ---")
            print("Escucharas clics metalicos mientras la aguja choca con el tope mecanico fisico.")
            print("Esto es normal y calibra la posicion de inicio.")
            input("Presiona Enter para comenzar...")
            for i in range(5):
                print(f"  [>] Forzando aguja hacia atras (Intento {i+1}/5)...")
                ser.write(b"EGT=-200\r\n")
                time.sleep(1.5)
            ser.write(b"EGT=0\r\n")
            print("[OK] Sincronizacion completada. La aguja esta ahora en el CERO fisico.")
            time.sleep(1.5)

        elif choice == "3":
            deg_in = input("Introduce temperatura deseada (C, rango 0 a 1100): ").strip()
            try:
                deg = float(deg_in)
                raw = int(deg / CALIBRATION_FACTOR)
                print(f"[*] Enviando: EGT={raw} (para {deg} C)")
                ser.write(f"EGT={raw}\r\n".encode('ascii'))
                time.sleep(1)
            except ValueError:
                print("Error: Valor de temperatura invalido.")
                time.sleep(1.5)

        elif choice == "4":
            print("[*] Apagando indicador APU EGT...")
            ser.write(b"EGT=OFF\r\n")
            time.sleep(1)

        elif choice == "0":
            break

# # --- TEST 3: LEDS INTERACTIVO ---
def test_leds_interactive(ser):
    clear_screen()
    print_header()
    print("\n--- TEST INTERACTIVO DE LEDS POR GRUPOS DE COLOR ---")
    print("Para no abrumarte confirmando uno a uno, encenderemos los LEDs por grupos.")
    print("Por favor, revisa el panel fisico y anota los codigos de los que NO enciendan.")
    print("--------------------------------------------------------------")
    input("Presiona Enter para comenzar el test de LEDs...")

    results = {}
    
    # Pre-cargar exclusiones de la Edicion Estandar como omitidas
    standard_exclusions = ["LDG_FLAP3_LAMP", "CREW_SUPPLY_SW", "PUMPS-L-TK-1-2"]
    for led in standard_exclusions:
        results[led] = "Omitido (No presente en Edicion Estandar)"

    # Definir grupos de color
    groups = {
        "AMBAR / CAUTION (Amarillos)": [led for led in LED_IDS if led in [
            "FPA1LP", "FPF1LP", "FPCLLP", "FPCRLP", "FPF2LP", "FPA2LP", "MBLED", "LOP", 
            "APUFAULT", "APUOVERSPEED", "LSOFF", "LTBOFF", "RSOFF", "RTBOFF", 
            "D1D", "SPOLED", "D2D", "YDOL", "HPAE1LP", "HPAE2LP", "HPBE1LP", 
            "HPBE2LP", "HPAOH", "HPBOH", "WHLSOH", "WHLFOH", "WHRFOH", "WHRSOH"
        ]],
        "AZUL / TRANSR & VALVE (Azules)": [led for led in LED_IDS if led in [
            "LGOB", "RGOB", "AGOB", "WAILVO", "WAIRVO", "EAILCVO", "EAIRCVO"
        ]],
        "VERDE / OTROS (Verdes/Otros)": [led for led in LED_IDS if led in [
            "WHLSO", "WHLFO", "WHRFO", "WHRSO", "EAILCAI", "EAIRCAI"
        ]]
    }

    try:
        # Apagar todos primero
        for led in LED_IDS:
            ser.write(f"{led}=OFF\r\n".encode('ascii'))
        time.sleep(0.5)

        for gname, gleds in groups.items():
            clear_screen()
            print_header()
            print(f"\nPROBANDO GRUPO: {gname} ({len(gleds)} LEDs)")
            print("--------------------------------------------------------------")
            print("LEDs encendidos en esta prueba:")
            for led in gleds:
                desc = LED_DESCRIPTIONS.get(led, "Sin descripcion")
                print(f"  - {led:<15} : {desc}")
            print("--------------------------------------------------------------")
            print("[*] Encendiendo grupo de LEDs...")
            for led in gleds:
                ser.write(f"{led}=ON\r\n".encode('ascii'))

            # Preguntar cuales fallaron
            while True:
                ans = input("\nIntroduce los codigos de los LEDs que NO encendieron (ej. LOP, FPA1LP)\nO presiona ENTER si todos encendieron correctamente: ").strip()
                if not ans:
                    # Todos OK en este grupo
                    for led in gleds:
                        results[led] = "OK"
                    break
                
                # Separar por comas
                raw_codes = [x.strip().upper() for x in ans.split(",") if x.strip()]
                
                # Validar codigos
                invalid_codes = [c for c in raw_codes if c not in gleds]
                if invalid_codes:
                    print(f"\n[!] Codigos incorrectos o de otro grupo: {', '.join(invalid_codes)}")
                    print("Por favor, introduce solo codigos de la lista superior.")
                    continue
                
                # Procesar
                for led in gleds:
                    if led in raw_codes:
                        results[led] = "FALLO (No se encendio)"
                    else:
                        results[led] = "OK"
                break

            # Apagar grupo de LEDs
            print("[*] Apagando grupo de LEDs...")
            for led in gleds:
                ser.write(f"{led}=OFF\r\n".encode('ascii'))
            time.sleep(0.5)

        # Mostrar resumen final del test de LEDs
        clear_screen()
        print_header()
        print("\n--- RESUMEN DEL TEST DE LEDS ---")
        ok_count = list(results.values()).count("OK")
        skipped_count = len(standard_exclusions)
        failed = {k: v for k, v in results.items() if v != "OK" and "Omitido" not in v}
        skipped = {k: v for k, v in results.items() if "Omitido" in v}

        print(f"LEDs Operativos: {ok_count} / {len(LED_IDS) - skipped_count} (Omitidos: {skipped_count})")
        if failed:
            print("\nSe han detectado los siguientes LEDs con fallos:")
            for k, v in failed.items():
                print(f"  - {k:<15} ({LED_DESCRIPTIONS.get(k, '')}): {v}")
        else:
            print("\nTodos los LEDs de la Edicion Estandar probados funcionan correctamente!")

        if skipped:
            print("\nLEDs omitidos (no presentes en la Edicion Estandar del panel B107):")
            for k in skipped.keys():
                print(f"  - {k:<15} ({LED_DESCRIPTIONS.get(k, '')})")

        input("\nPresiona Enter para regresar al menu principal...")

    except KeyboardInterrupt:
        # Asegurarse de apagar todos
        for led in LED_IDS:
            ser.write(f"{led}=OFF\r\n".encode('ascii'))
        print("\n[!] Test interrumpido. Todos los LEDs han sido apagados.")
        time.sleep(1.5)


# --- TEST 4: RETROILUMINACION (BACKLIGHT) ---
def test_backlight(ser):
    while True:
        clear_screen()
        print_header()
        print("\n--- TEST DE RETROILUMINACION (BACKLIGHT) ---")
        print("El panel Rowsfire B107 regula el brillo global de las placas con la clave 'LED'.")
        print("Selecciona una opcion de prueba:\n")
        print("1. Encender al 100% (LED=100.0000)")
        print("2. Encender al 50% (LED=50.0000)")
        print("3. Encender al 20% (LED=20.0000)")
        print("4. Apagar retroiluminacion (LED=0.0000 o LED=OFF)")
        print("5. Ciclo de Desvanecimiento Automatico (Fading 0% -> 100% -> 0%)")
        print("6. Introducir Valor de Brillo Personalizado (0.0 a 100.0)")
        print("0. Regresar al menu principal")
        print("--------------------------------------------------------------")
        
        choice = input("Selecciona una opcion: ").strip()
        
        if choice == "1":
            print("[*] Enviando LED=100.0000...")
            ser.write(b"LED=100.0000\r\n")
            time.sleep(0.5)
        elif choice == "2":
            print("[*] Enviando LED=50.0000...")
            ser.write(b"LED=50.0000\r\n")
            time.sleep(0.5)
        elif choice == "3":
            print("[*] Enviando LED=20.0000...")
            ser.write(b"LED=20.0000\r\n")
            time.sleep(0.5)
        elif choice == "4":
            print("[*] Enviando LED=OFF...")
            ser.write(b"LED=OFF\r\n")
            ser.write(b"LED=0.0000\r\n")
            time.sleep(0.5)
        elif choice == "5":
            print("\n[*] Iniciando ciclo automatico de fading. Presiona Ctrl+C para interrumpir...")
            try:
                # Subir de 0 a 100%
                for i in range(0, 101, 10):
                    val = f"LED={i:.4f}\r\n"
                    print(f"  [>] Brillo: {i}% (Comando: {val.strip()})", end="\r")
                    ser.write(val.encode('ascii'))
                    time.sleep(0.15)
                # Bajar de 100 a 0%
                for i in range(100, -1, -10):
                    val = f"LED={i:.4f}\r\n"
                    print(f"  [>] Brillo: {i}% (Comando: {val.strip()})", end="\r")
                    ser.write(val.encode('ascii'))
                    time.sleep(0.15)
                print("\n[OK] Ciclo de retroiluminacion terminado.")
                time.sleep(1)
            except KeyboardInterrupt:
                print("\n[!] Ciclo cancelado por el usuario.")
                ser.write(b"LED=OFF\r\n")
                time.sleep(1)
        elif choice == "6":
            val_in = input("Introduce porcentaje de brillo (0.0 a 100.0): ").strip()
            try:
                val_float = float(val_in)
                if 0 <= val_float <= 100:
                    cmd = f"LED={val_float:.4f}\r\n"
                    print(f"[*] Enviando: {cmd.strip()}")
                    ser.write(cmd.encode('ascii'))
                    time.sleep(0.5)
                else:
                    print("Error: El valor debe estar entre 0 y 100.")
                    time.sleep(1.5)
            except ValueError:
                print("Error: El valor debe ser numerico.")
                time.sleep(1.5)
        elif choice == "0":
            break


# --- TEST 5: INTERRUPTORES Y DETECCION DE POSICIONES (SIMPLIFICADO) ---
def draw_switches_total_dashboard(detected_positions, last_event=""):
    """Dibuja el estado simplificado de deteccion de todos los switches en columnas"""
    clear_screen()
    print_header()
    print("\n==============================================================")
    print("           TEST DE DETECCION DE INTERRUPTORES (SWITCHES)      ")
    print(" Mueve los interruptores fisicos para comprobar sus posiciones.")
    print(" Presiona la tecla ENTER en tu teclado para finalizar el test. ")
    print("==============================================================")
    
    # Agrupar por switch para ver cuales estan pendientes
    pending_items = []
    total_positions = 0
    detected_count = 0
    
    for category, switches in SWITCHES_SPEC.items():
        for sw, positions in switches.items():
            missing_pos = []
            for pos in positions:
                total_positions += 1
                if detected_positions.get((sw, pos), False):
                    detected_count += 1
                else:
                    missing_pos.append(pos)
            if missing_pos:
                pos_str = ",".join(missing_pos)
                pending_items.append(f"{sw} ({pos_str})")
                
    if last_event:
        print(f" Ultimo evento detectado: {last_event}")
    else:
        print(f" Ultimo evento detectado: ---")
        
    print(f" Progreso General: {detected_count}/{total_positions} posiciones mapeadas")
    print(f" Switches pendientes: {len(pending_items)}")
    print("==============================================================")
    
    if not pending_items:
        print("\n   !!! TODOS LOS SWITCHES Y POSICIONES DETECTADOS !!! \n")
    else:
        # Imprimir en 3 columnas para optimizar espacio
        col_width = 25
        cols = 3
        for i in range(0, len(pending_items), cols):
            chunk = pending_items[i:i+cols]
            row_str = ""
            for item in chunk:
                if len(item) > col_width - 2:
                    item_disp = item[:col_width - 5] + "..."
                else:
                    item_disp = item
                row_str += f"  {item_disp:<{col_width}}"
            print(row_str)
            
    print("==============================================================")


def test_switches(ser, detected_positions):
    """Ejecuta la captura interactiva simplificada para todos los switches a la vez"""
    last_event_str = ""
    draw_switches_total_dashboard(detected_positions, last_event_str)

    while True:
        # Comprobar si el usuario presiono ENTER para salir
        if msvcrt_available:
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key in [b'\r', b'\n']:
                    break
        else:
            input("\nPresiona Enter para terminar el test...")
            break

        # Leer puerto serie
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                events = parse_line(line)
                if events:
                    changed = False
                    for sw, pos in events:
                        key_tuple = (sw, pos)
                        if key_tuple in detected_positions:
                            if not detected_positions[key_tuple]:
                                detected_positions[key_tuple] = True
                                changed = True
                            last_event_str = f"{sw} = {pos}"
                        else:
                            # Capturar interruptor dinamico extra no listado en el SPEC
                            detected_positions[key_tuple] = True
                            changed = True
                            last_event_str = f"{sw} = {pos}"
                            
                    if changed or last_event_str:
                        draw_switches_total_dashboard(detected_positions, last_event_str)
            except Exception as e:
                print(f"[!] Error leyendo puerto serie: {e}")
                break
        time.sleep(0.02)
        
    # Mostrar reporte final
    show_switches_final_summary(detected_positions)


def show_switches_final_summary(detected_positions):
    """Muestra un reporte detallado de los switches y estados que faltaron por probar"""
    clear_screen()
    print_header()
    print("\n--- INFORME FINAL DE INTERRUPTORES PROBADOS ---")

    # Filtrar solo elementos del mapa oficial
    official_keys = []
    for category, switches in SWITCHES_SPEC.items():
        for sw, positions in switches.items():
            for pos in positions:
                official_keys.append((sw, pos))

    total_official = len(official_keys)
    detected_official = sum(1 for k in official_keys if detected_positions.get(k, False))
    percent = (detected_official / total_official) * 100 if total_official > 0 else 0

    print(f"Deteccion total de estados oficiales: {detected_official}/{total_official} ({percent:.1f}%)")
    print("--------------------------------------------------------------")

    # Agrupar los faltantes por categoria
    for category, switches in SWITCHES_SPEC.items():
        cat_total = 0
        cat_detected = 0
        cat_missing = []

        for sw, positions in switches.items():
            for pos in positions:
                cat_total += 1
                if detected_positions.get((sw, pos), False):
                    cat_detected += 1
                else:
                    cat_missing.append(f"{sw}={pos}")

        print(f"\nCategoria [{category}]: {cat_detected}/{cat_total} detectados")
        if cat_missing:
            print("  Estados faltantes por registrar:")
            # Agrupar por switch para hacerlo mas corto
            sw_missing = {}
            for m in cat_missing:
                sw, pos = m.split("=")
                sw_missing.setdefault(sw, []).append(pos)
            
            for sw, missing_states in list(sw_missing.items())[:5]:
                print(f"    - {sw}: falta {', '.join(missing_states)}")
            if len(sw_missing) > 5:
                print(f"    - ... y otros {len(sw_missing) - 5} switches mas.")
        else:
            print("  [OK] Enhorabuena! Todos los estados de esta categoria fueron detectados.")

    input("\nPresiona Enter para regresar al menu principal...")


# --- MONITOR DE EVENTOS SERIE RAW (DEPURACION) ---
def test_raw_serial(ser):
    clear_screen()
    print_header()
    print("\n--- MONITOR DE EVENTOS SERIE RAW (DEPURACION) ---")
    print("Mueve cualquier control en el panel. Veras la trama exacta que envia.")
    print("Esto es util para detectar si un switch fisico esta fallando o no envia datos.")
    print("Presiona ENTER en tu teclado para regresar al menu.")
    print("--------------------------------------------------------------")
    
    # Consumir datos viejos
    ser.reset_input_buffer()
    
    while True:
        if msvcrt_available:
            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key in [b'\r', b'\n']:
                    break
        else:
            input("\nPresiona Enter para salir...")
            break
            
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line:
                    print(f"  [RAW] {line}")
            except Exception as e:
                print(f"[ERROR] {e}")
                break
        time.sleep(0.01)


# --- MENU PRINCIPAL ---
def main():
    global HW_PORT
    
    # Inicializar base de datos de posiciones detectadas en la sesion
    detected_positions = {}
    for category, switches in SWITCHES_SPEC.items():
        for sw, positions in switches.items():
            for pos in positions:
                detected_positions[(sw, pos)] = False

    while True:
        clear_screen()
        print_header()
        print(f"\nPuerto de conexion configurado: {HW_PORT}")
        print("Estado del Hardware: DESCONECTADO (Selecciona la opcion 1 para conectar)")
        print("--------------------------------------------------------------")
        print("1. Conectar a Panel B107 e iniciar diagnosticos")
        print("2. Cambiar Puerto COM actual")
        print("0. Salir de la aplicacion")
        print("--------------------------------------------------------------")
        
        choice = input("Selecciona una opcion: ").strip()
        
        if choice == "1":
            print(f"\n[*] Abriendo conexion en {HW_PORT} a {BAUD_RATE} bps...")
            try:
                with serial.Serial(HW_PORT, BAUD_RATE, timeout=0.1) as ser:
                    print("[*] Puerto abierto. Sincronizando con Arduino...")
                    time.sleep(1.5)
                    ser.reset_input_buffer()
                    
                    # Handshake
                    ser.write(b"model_flag\r\n")
                    time.sleep(0.5)
                    response = ser.readline().decode('ascii', errors='ignore').strip()
                    print(f"[<] Respuesta del hardware: {response}")
                    
                    # Cargar estado inicial (Snapshot) enviando comando STATE
                    print("[*] Solicitando snapshot de estado inicial (STATE)...")
                    ser.write(b"STATE\r\n")
                    time.sleep(0.5)
                    
                    # Leer y parsear el snapshot inicial
                    if ser.in_waiting > 0:
                        snap_line = ser.readline().decode('ascii', errors='ignore').strip()
                        events = parse_line(snap_line)
                        for sw, pos in events:
                            key_tuple = (sw, pos)
                            if key_tuple in detected_positions:
                                detected_positions[key_tuple] = True
                        print("[OK] Estado inicial sincronizado.")
                        time.sleep(0.8)
                    else:
                        print("[AVISO] No se recibio snapshot inicial, pero la conexion continua.")
                        time.sleep(1)

                    # Submenu interactivo con el hardware conectado
                    while True:
                        clear_screen()
                        print_header()
                        print(f"\nConectado a Panel B107 en {HW_PORT}")
                        print("--------------------------------------------------------------")
                        print("1. Test de Displays de Altitud (FLT/LAND ALT)")
                        print("2. Test de Aguja APU EGT")
                        print("3. Test de LEDs por Grupos de Color (Ambar / Azul / Verde)")
                        print("4. Test de Retroiluminacion (Backlight del panel)")
                        print("5. Test de Interruptores e Ingesta de Eventos (Switches)")
                        print("6. Monitor de Eventos Serie RAW (Depuracion de Switches)")
                        print("0. Desconectar y salir al menu principal")
                        print("--------------------------------------------------------------")
                        
                        sub_choice = input("Selecciona una opcion: ").strip()
                        
                        if sub_choice == "1":
                            test_displays(ser)
                        elif sub_choice == "2":
                            test_egt(ser)
                        elif sub_choice == "3":
                            test_leds_interactive(ser)
                        elif sub_choice == "4":
                            test_backlight(ser)
                        elif sub_choice == "5":
                            test_switches(ser, detected_positions)
                        elif sub_choice == "6":
                            test_raw_serial(ser)
                        elif sub_choice == "0":
                            print("\n[*] Cerrando conexion serie...")
                            break
                        else:
                            print("Opcion invalida.")
                            time.sleep(1)
            except serial.SerialException as e:
                print(f"\n[ERROR] No se pudo conectar al puerto {HW_PORT}: {e}")
                print("Verifica si el cable esta conectado o si el puerto esta siendo usado por otra app.")
                input("\nPresiona Enter para regresar...")
                
        elif choice == "2":
            clear_screen()
            print_header()
            print("\n--- CAMBIAR CONFIGURACION DE PUERTO ---")
            print(f"Puerto actual: {HW_PORT}")
            port_input = input("Introduce el puerto COM del Arduino (ej. COM3) o presiona Enter para cancelar: ").strip().upper()
            if port_input:
                HW_PORT = port_input
                print(f"[OK] Puerto cambiado a {HW_PORT}.")
            time.sleep(1)
            
        elif choice == "0":
            print("\nGracias por usar la herramienta! Buenos vuelos. (B737 Overhead)")
            sys.exit(0)

if __name__ == "__main__":
    main()
