#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

typedef struct struct_remote_control {
  int motorID;      // ID do motor (1-4)
  int velocidade;   // PWM para controle de velocidade (0 a 255)
  int sentido;      // Sentido do motor (0: Horário, 1: Anti-horário)
  int brake;         // 
} struct_remote_control;

struct_remote_control remoteControl; // Struct para armazenar os dados do controle remoto

// Variáveis para MAC Address do dispositivo receptor
uint8_t broadcastAddress[] = {0xE8, 0x6B, 0xEA, 0xD0, 0xE4, 0xF0};  // Substitua pelo MAC Address real

// Pinos dos botões
int btnCima = 27;    
int btnBaixo = 25;
int btnEsquerda = 33;
int btnDireita = 14;
int btnEsqMaisRapido = 5;
int btnDirMaisRapido = 12;

// Pino do LED
int ledPin = 2;

// Estados atuais e anteriores dos botões
bool lastBtnCima = HIGH;
bool lastBtnBaixo = HIGH;
bool lastBtnEsquerda = HIGH;
bool lastBtnDireita = HIGH;
bool lastBtnEsqMaisRapido = HIGH;
bool lastBtnDirMaisRapido = HIGH;

// Variável para armazenar o status do último envio
bool envioSucesso = false;

// Função de callback de sucesso no envio
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nStatus do envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");

  // Atualiza o status do envio
  envioSucesso = (status == ESP_NOW_SEND_SUCCESS);

  // Desliga o LED quando o envio termina (independente do sucesso ou falha)
  digitalWrite(ledPin, LOW);
}

// Inicializa ESP-NOW
void setupESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  // Adiciona o par (peer) para ESP-NOW
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erro ao adicionar peer");
    return;
  }
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  // Inicializa o Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin();

  // Inicializa ESP-NOW
  setupESPNow();
  
  esp_wifi_set_channel(0, WIFI_SECOND_CHAN_NONE);



  // Configura os pinos dos botões como entradas
  pinMode(btnCima, INPUT_PULLUP);
  pinMode(btnBaixo, INPUT_PULLUP);
  pinMode(btnEsquerda, INPUT_PULLUP);
  pinMode(btnDireita, INPUT_PULLUP);
  pinMode(btnEsqMaisRapido, INPUT_PULLUP);
  pinMode(btnDirMaisRapido, INPUT_PULLUP);

  // Configura o pino do LED como saída
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // LED desligado inicialmente
}

void enviarComando(int motorID, int sentido, int velocidade, int brake) {
  remoteControl.motorID = motorID;
  remoteControl.sentido = sentido;
  remoteControl.velocidade = velocidade;
  remoteControl.brake = brake;
  
  // Liga o LED enquanto envia a mensagem
  digitalWrite(ledPin, HIGH);

  // Reinicia o status de envio
  envioSucesso = false;

  // Loop que continua tentando enviar até que o envio seja bem-sucedido
  while (!envioSucesso) {
    esp_now_send(broadcastAddress, (uint8_t *) &remoteControl, sizeof(remoteControl));
    delay(0);  // Pequeno atraso para evitar que o envio seja muito rápido
  }
}

void verificarBotao(int pin, bool &ultimoEstado, int motorID, int sentido, int velocidade) {
  bool estadoAtual = digitalRead(pin);
  
  // Botão pressionado
  if (estadoAtual == LOW && ultimoEstado == HIGH) {
    enviarComando(motorID, sentido, velocidade, 0);
  }
  
  // Botão solto
  if (estadoAtual == HIGH && ultimoEstado == LOW) {
    enviarComando(motorID, sentido, 0, 1);  // Envia comando com velocidade 0
  }
  
  // Atualiza o estado anterior
  ultimoEstado = estadoAtual;
}

void loop() {
  // Verifica cada botão e envia comandos conforme o estado
  verificarBotao(btnCima, lastBtnCima, 1, 0, 128);  // Motor 1, sentido horário, velocidade média
  verificarBotao(btnBaixo, lastBtnBaixo, 1, 1, 128);  // Motor 1, sentido anti-horário, velocidade média
  verificarBotao(btnEsquerda, lastBtnEsquerda, 1, 0, 0);  // Motor 1, sentido horário, velocidade 0
  verificarBotao(btnDireita, lastBtnDireita, 2, 0, 128);  // Motor 2, sentido horário, velocidade média
  verificarBotao(btnEsqMaisRapido, lastBtnEsqMaisRapido, 3, 0, 255);  // Motor 3, sentido horário, velocidade máxima
  verificarBotao(btnDirMaisRapido, lastBtnDirMaisRapido, 3, 1, 255);  // Motor 3, sentido anti-horário, velocidade máxima  
}
