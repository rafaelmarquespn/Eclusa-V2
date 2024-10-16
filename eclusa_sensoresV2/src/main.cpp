
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

//mac: E8:6B:EA:D0:E4:F0 sender
//mac: 08:D1:F9:27:B2:54 reciever

// MAC address of the receiver
// uint8_t broadcastAddress[] = {0xE8, 0x6B, 0xEA, 0xD0, 0xE4, 0xF0};
uint8_t broadcastAddress[] = {0xE8, 0x6B, 0xEA, 0xD0, 0xE4, 0xF0};

// Pinos dos sensores nomeados de X1 a X18
const int sensorPins[18] = {15, 2, 4, 5, 18, 19, 22, 23, 13, 13, 14, 27, 26, 25, 33, 32, 35, 34};

// Structure to send data
typedef struct struct_message {
  bool sensorStatus[18]; // Array para armazenar o status ON/OFF dos sensores
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// Callback quando os dados são enviados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Função para ler o status dos sensores
void readSensors() {
  for (int i = 0; i < 18; i++) {
    // Ler o status do sensor conectado ao pino correspondente
    myData.sensorStatus[i] = digitalRead(sensorPins[i]) == HIGH; // HIGH = ON, LOW = OFF
  }
}

void setup() {
  // Inicializar o monitor serial
  Serial.begin(115200);
  Serial.println("Eclusa Sensores 2.0");

  // Configurar os pinos dos sensores como entrada
  for (int i = 0; i < 18; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Configurar o dispositivo como estação Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin();

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Registrar callback para o status de envio
  esp_now_register_send_cb(OnDataSent);

  // Registrar peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Adicionar peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Ler o status dos sensores
  readSensors();
  
  // Enviar mensagem via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  Serial.println("Status dos sensores:");
  for (int i = 0; i < 18; i++) {
    Serial.print("Sensor X");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(myData.sensorStatus[i]);
  }
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  // Delay para evitar envio contínuo
  delay(20);
}
