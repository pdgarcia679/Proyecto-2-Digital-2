#include <esp_now.h>
#include <WiFi.h>

#define RX2 16  
#define TX2 17  // TX2 para enviar por UART2

typedef struct mensaje {
  char a[2];
} mensaje;

mensaje datos;

void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* datosRecibidos, int len) {
  memcpy(&datos, datosRecibidos, sizeof(datos));

  // Mostrar por monitor serial 
  Serial.print("Recibido: ");
  Serial.println(datos.a);

  // Enviar por UART2
  Serial2.print(datos.a);
}

void setup() {
  Serial.begin(115200);                      
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2); // UART2

  WiFi.mode(WIFI_STA); // Modo estación para ESP-NOW

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al iniciar ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv); // Callback al recibir datos
}

void loop() {
  
}

