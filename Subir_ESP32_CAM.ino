#include "esp_camera.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// =====================================================
// Configuración de pines para la cámara (módulo AI-Thinker)
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// =====================================================
// Credenciales WiFi
const char* ssid = "tu nombre de red";
const char* password = "tu contraseña";

// =====================================================
// Pines de motores (PWM)
// Se usarán pines 14 (motor derecho) y 15 (motor izquierdo)
// Se emplea ledcWrite para generar la salida PWM (equivalente a analogWrite)
#define MOTOR_RIGHT_PIN 14
#define MOTOR_LEFT_PIN 15

// Configuración de PWM: dos canales, 5 kHz y resolución de 8 bits (0-255)
const int pwmChannelRight = 0;
const int pwmChannelLeft = 1;
const int freq = 5000;
const int resolu = 8;

// =====================================================
// Servidores:
// - Endpoint de control en el puerto 80
// - Servidor de streaming en el puerto 81
AsyncWebServer serverControl(80);
WiFiServer serverStream(81);

// =====================================================
// Función para inicializar la cámara
void initCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 30;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK){
    Serial.printf("Error al inicializar la cámara: 0x%x", err);
    while(1) delay(1000);
  }
}

// =====================================================
// Tarea para manejar el streaming (puerto 81)
void streamTask(void * parameter){
  for(;;){
    WiFiClient client = serverStream.available();
    if(client){
      Serial.println("Cliente de stream conectado");
      client.print("HTTP/1.1 200 OK\r\n");
      client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
      
      while(client.connected()){
        camera_fb_t * fb = esp_camera_fb_get();
        if(!fb){
          Serial.println("Error al capturar frame");
          continue;
        }
        String header = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
        client.print(header);
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        esp_camera_fb_return(fb);
        delay(10); // Ajusta la tasa de frames si es necesario
      }
      client.stop();
      Serial.println("Cliente de stream desconectado");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// =====================================================
void setup(){
  Serial.begin(115200);
  
  // Configuración de PWM para los motores
  ledcSetup(pwmChannelRight, freq, resolu);
  ledcAttachPin(MOTOR_RIGHT_PIN, pwmChannelRight);
  ledcSetup(pwmChannelLeft, freq, resolu);
  ledcAttachPin(MOTOR_LEFT_PIN, pwmChannelLeft);
  
  // Inicia con motores detenidos
  ledcWrite(pwmChannelRight, 0);
  ledcWrite(pwmChannelLeft, 0);
  
  // Inicializa la cámara
  initCamera();
  
  // Conexión a WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  Serial.println(WiFi.localIP());
  
  // Endpoint para establecer la velocidad de los motores
  // Se espera recibir dos parámetros: "left" y "right" (valores entre 0 y 255)
  serverControl.on("/setMotor", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("left") && request->hasParam("right")){
      String leftStr = request->getParam("left")->value();
      String rightStr = request->getParam("right")->value();
      int leftSpeed = leftStr.toInt();
      int rightSpeed = rightStr.toInt();
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      ledcWrite(pwmChannelLeft, leftSpeed);
      ledcWrite(pwmChannelRight, rightSpeed);
      Serial.printf("Velocidades actualizadas: Izquierda = %d, Derecha = %d\n", leftSpeed, rightSpeed);
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Faltan parámetros");
    }
  });
  
  // Inicia el servidor de control (puerto 80)
  serverControl.begin();
  // Inicia el servidor de streaming (puerto 81)
  serverStream.begin();
  
  // Crea una tarea para el streaming en un núcleo separado
  xTaskCreatePinnedToCore(
      streamTask,      // Función de la tarea
      "StreamTask",    // Nombre de la tarea
      8192,            // Tamaño del stack en bytes
      NULL,            // Parámetro de la tarea
      1,               // Prioridad
      NULL,            // Manejador de la tarea
      1                // Núcleo en el que se ejecuta
  );
}

void loop(){
  // No se requiere código en loop
}
