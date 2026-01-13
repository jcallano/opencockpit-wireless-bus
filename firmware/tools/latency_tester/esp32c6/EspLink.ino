#include "EspLink.h"
#include <esp_mac.h>

// ======================================================
// CONFIGURACIÓN (REEMPLAZA CON LA MAC DEL OTRO EQUIPO)
// ======================================================
uint8_t otherDeviceMac[] = {0xDC, 0xDA, 0x0C, 0x5A, 0x83, 0x64}; 

EspLink miEnlace;

// --- Callback: Datos ---
void alRecibirDatos(const char* msg) {
  Serial.print("[APP] Mensaje recibido: ");
  Serial.println(msg);
}

// --- Callback: Latencia ---
void alRecibirLatencia(unsigned long micros) {
  Serial.print("[APP] Latencia (Ida y vuelta): ");
  Serial.print(micros);
  Serial.println(" us");
}

void setup() {
  // Intentar CPU al máximo (S3 sube a 240, C6 se queda en 160)
  setCpuFrequencyMhz(240); 

  Serial.begin(115200);
  delay(1000);

  Serial.println("--- INICIANDO SISTEMA ESP-NOW OPTIMIZADO ---");

  // Nota: Ya no necesitamos WiFi.mode() ni leer la MAC aquí manualmente.
  // La clase EspLink se encarga de configurar el modo AP_STA y el canal.
  
  // Imprimimos la MAC real del hardware para referencia
  // Usamos la función nativa que lee directo del efuse (funciona siempre)
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.printf("MI MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                baseMac[0], baseMac[1], baseMac[2], 
                baseMac[3], baseMac[4], baseMac[5]);

  // Iniciar la clase
  miEnlace.begin(otherDeviceMac);

  // Asignar callbacks
  miEnlace.onDataReceived = alRecibirDatos;
  miEnlace.onLatencyAvailable = alRecibirLatencia;
}

void loop() {
  static unsigned long lastTime = 0;
  
  // Enviar cada 1 segundo (1000 ms)
  if (millis() - lastTime > 10) {
    lastTime = millis();

    // 1. Enviar dato útil
    miEnlace.send("Hola desde clase!");

    // 2. Medir latencia (Ping ligero de 1 byte)
    miEnlace.measureLatency();
  }
}