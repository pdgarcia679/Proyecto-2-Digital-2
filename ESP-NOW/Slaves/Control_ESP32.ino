#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// pantalla
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
// neopixel
#define PIN 2
#define NUMPIXELES 1

Adafruit_NeoPixel pixels(NUMPIXELES, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pines de botones
#define BTN_A 26
#define BTN_B 23
#define BTN_JOY 25  // botón del joystick

#define SW1 15
#define SW2 4
#define BTN_RIGHT 16
#define BTN_UP 17
#define BTN_SELECT 5
#define BTN_DOWN 18
#define BTN_LEFT 19


// Pines del joystick analógico
#define JOY_VRY 32
#define JOY_VRX 33


// MAC de la placa 
uint8_t macReceptora[] = { 0x30, 0xAE, 0xA4, 0xF2, 0x5B, 0x30 };

// para envio de datos
typedef struct mensaje {
  char a[2];
} mensaje;

mensaje datos;

// Notifica si los datos son recibidos
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nEstado del último paquete enviado:\t");

  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Entregado");

  } else {
    Serial.println("Ha ocurrido un fallo en la entrega");

  }
}

// Estado anterior de botones
bool lastA = true, lastB = true, lastSelect = true;
bool lastRight = true, lastUp = true, lastDown = true, lastLeft = true;
bool lastJoyBtn = true;  // estado del botón del joystick

// Dirección anterior del joystick
String lastJoystickDir = "";

void setup() {
  Serial.begin(115200);

  // pantalla
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C es la dirección I2C típica
    Serial.println(F("No se detectó la pantalla OLED"));
    while (true)
      ;  
  }

  //  Neopixel
  pixels.begin();
  pixels.setBrightness(5);  

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();


  // pull-up
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_JOY, INPUT_PULLUP);  

  // Configuración WiFi modo ESTACIÓN
  WiFi.mode(WIFI_STA);

  // Inicio de comunicación ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Ha ocurrido un error al iniciar ESP-NOW");
    return;
  }

  
  esp_now_register_send_cb(OnDataSent);

  
  esp_now_peer_info_t datosEmparejamiento;
  memcpy(datosEmparejamiento.peer_addr, macReceptora, 6);
  datosEmparejamiento.channel = 0;
  datosEmparejamiento.encrypt = false;

  // receptor 
  if (esp_now_add_peer(&datosEmparejamiento) != ESP_OK) {
    Serial.println("Fallo de emparejamiento");
    return;
  }

  Serial.println("Emisor listo para enviar datos.");
  display.clearDisplay();
  display.setTextSize(2);               // Tamaño de texto
  display.setTextColor(SSD1306_WHITE);  
  display.setCursor(0, 0);
  display.println("Control");
  display.setCursor(0, 35);
  display.println("Listo");
  display.display();  // Muestra en pantalla
}

void loop() {
  // Lectura de botones

  checkButton(BTN_A, lastA, "y");
  checkButton(BTN_B, lastB, "z");
  checkButton(BTN_SELECT, lastSelect, "q");
  checkButton(BTN_RIGHT, lastRight, "f");
  checkButton(BTN_UP, lastUp, "t");
  checkButton(BTN_DOWN, lastDown, "v");
  checkButton(BTN_LEFT, lastLeft, "h");
  checkButton(BTN_JOY, lastJoyBtn, "e");  // nuevo botón
  checkJoystick();
  delay(100);
}

void checkButton(uint8_t pin, bool &lastState, const char *name) {
  bool currentState = digitalRead(pin);

  if (lastState == HIGH && currentState == LOW) {
    Serial.println(name);

    // Mostrar botón presionado en pantalla
    mostrarEnPantalla(name);

    // Valores de los datos a enviar
    strcpy(datos.a, name);

    // Envío de mensaje por ESP-NOW
    esp_err_t resultado = esp_now_send(macReceptora, (uint8_t *)&datos, sizeof(datos));

    if (resultado == ESP_OK) {
      Serial.println("Enviado correctamente");
    } else {
      Serial.println("Ha ocurrido un error en el envío de datos");
    }
  }

  lastState = currentState;
}


void checkJoystick() {
  //WiFi.mode(WIFI_OFF);
  int x = analogRead(JOY_VRX);
  int y = analogRead(JOY_VRY);
  //WiFi.mode(WIFI_STA);

  String direction = "";

  const int thresholdLow = 1200;
  const int thresholdHigh = 2800;

  if (x < thresholdLow) direction = "2";
  else if (x > thresholdHigh) direction = "8";
  else if (y < thresholdLow) direction = "4";
  else if (y > thresholdHigh) direction = "6";
  else direction = "5";

  if (direction != lastJoystickDir) {
    Serial.println("Joystick: " + direction);
    // Valores de los datos a enviar
    strcpy(datos.a, direction.c_str());

    // Envío de mensaje por ESP-NOW
    esp_err_t resultado = esp_now_send(macReceptora, (uint8_t *)&datos, sizeof(datos));

    if (resultado == ESP_OK) {
      Serial.println("Enviado correctamente");
    } else {
      Serial.println("Ha ocurrido un error en el envío de datos");
    }

    lastJoystickDir = direction;
  }
}
void mostrarEnPantalla(const char *texto) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println("Boton:");
  display.setCursor(0, 45);
  display.println(texto);
  display.display();
}
