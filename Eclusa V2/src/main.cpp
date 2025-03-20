#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ModbusRTUSlave.h>
#include <LittleFS.h>
//#include <AsyncElegantOTA.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>




#define CHANNEL 0 // Canal ESP-NOW para comunicação
// Estrutura para armazenar os dados recebidos via ESP-NOW
struct __attribute__((packed)) Message {
  int pwm;
  int sentido;
};

//wifi
const char *ssid = "Moto G (5) Plus 3746";
const char *senha = "velorio_";
WiFiClient espClient;
boolean conectado = false;
int intervalo = 5000;
unsigned long tempo = millis(); 

const int motor1 = 23 ; 
const int motor2 = 22; 
const int motor3 =21 ; 
const int motor4 = 19;

const int freio1 = 2 ;
const int freio2 = 4 ;
const int freio3 = 5 ; 
const int freio4 = 18 ;

const int sentidoM1 = 13;
const int sentidoM2 = 12;
const int sentidoM3 = 14;
const int sentidoM4 = 27;

const int manta = 26;

//Arquivos
String index_html;
String arquivoConf;

//Objetos
AsyncWebServer server(80);
DynamicJsonDocument doc(1024);

// put function declarations here:
void setup_pinos();
void aciona_motor(int motor, int velocidade, int sentido);
void setupServer();
void handleConfigJson(AsyncWebServerRequest *request);
void saveJson();
void setupWiFi();
void scanWiFiNetworks(AsyncWebServerRequest *request);
void handleWiFiStatus(AsyncWebServerRequest *request);


void setup() {
  setup_pinos();
    //WiFi
  WiFi.begin(ssid, senha);
  Serial.println(WiFi.localIP());

    if (!LittleFS.begin()) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  // abre o arquivo index.html
  File file = LittleFS.open("/index.html", "r");
  index_html = file.readString();
  if(!file){
    Serial.println("Failed to open index for reading");
    return;
  }
  file.close();
  // abre o arquivo json com as configs
  File configFile=LittleFS.open("/config.json", "r");
  arquivoConf=configFile.readString();
  deserializeJson(doc, arquivoConf);
  JsonObject configs = doc.as<JsonObject>();   // converte em json o arquivo das configs
  if (configs.isNull()) {
    Serial.println("Falha ao carregar o arquivo de configuração");
    return;
  }
  ssid = configs["wifiSSID"];   //carrega os parametros que estão no json
  senha = configs["wifiPassword"];
  configFile.close();
  
  //Server
  setupServer();
  server.begin();

  }

void loop() {



  if (WiFi.status() != WL_CONNECTED && millis() - tempo > intervalo){
    setupWiFi();
    tempo=millis();
  }

}

//Função para acionar os motores e desativar os freios
void aciona_motor(int motor, int velocidade, int sentido){
  switch (motor)
  {
  case 1:
    digitalWrite(sentidoM1, sentido);
    digitalWrite(freio1, HIGH);
    ledcWrite(0, velocidade);
    break;

  case 2:
    digitalWrite(sentidoM2, sentido);
    digitalWrite(freio2, HIGH);
    ledcWrite(1, velocidade);
    break;
  
  case 3:
    digitalWrite(sentidoM3, sentido);
    digitalWrite(freio3, HIGH);
    ledcWrite(2, velocidade);
    break;
  
  case 4:
    digitalWrite(sentidoM4, sentido);
    digitalWrite(freio4, HIGH);
    ledcWrite(3, velocidade);
    break;
  
  default:
    break;
  }
}

//Configura todos os pinos
void setup_pinos(){
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  pinMode(freio1, OUTPUT);
  pinMode(freio2, OUTPUT);
  pinMode(freio3, OUTPUT);
  pinMode(freio4, OUTPUT);

  pinMode(sentidoM1, OUTPUT);
  pinMode(sentidoM2, OUTPUT);
  pinMode(sentidoM3, OUTPUT);
  pinMode(sentidoM4, OUTPUT);
  
  pinMode(manta, OUTPUT);
  //Configuração do PWM
  ledcSetup(0, 300, 8);          // Configuração do canal de PWM (0)
  ledcAttachPin(motor1, 0);       // Vincula o canal de PWM (0) ao pino do motor
  ledcWrite(0, 0);           // Define o PWM inicial do motor

  ledcSetup(0, 300, 8);          // Configuração do canal de PWM (0)
  ledcAttachPin(motor2, 1);       // Vincula o canal de PWM (0) ao pino do motor
  ledcWrite(0, 0);  

  ledcSetup(0, 300, 8);          // Configuração do canal de PWM (0)
  ledcAttachPin(motor3, 2);       // Vincula o canal de PWM (0) ao pino do motor
  ledcWrite(0, 0);  

  ledcSetup(0, 300, 8);          // Configuração do canal de PWM (0)
  ledcAttachPin(motor4, 3);       // Vincula o canal de PWM (0) ao pino do motor
  ledcWrite(0, 0);
} 

//Configuração do JSON
void handleConfigJson(AsyncWebServerRequest *request) {
  // Cria um objeto JSON
  StaticJsonDocument<200> doc;
  // Define os valores no JSON
  doc["wifiSSID"] = ssid;
  doc["wifiPassword"] = senha;
  // Converte o JSON em uma string
  String json;
  serializeJson(doc, json);
  // Envia a resposta com o tipo de conteúdo JSON
  request->send(200, "application/json", json);
}

//Salva o JSON
void saveJson(){
  File configFile=LittleFS.open("/config.json", "w");
  // Cria um objeto JSON
  StaticJsonDocument<200> doc;
  // Define os valores no JSON
  doc["wifiSSID"] = ssid;
  doc["wifiPassword"] = senha;
  // Converte o JSON em uma string
  String json;
  serializeJson(doc, json);
  configFile.print(json);
  configFile.close();
}

//Configuração do servidor
void setupServer(){
      //Página de configuração
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");

  });
  server.on("/WIFI", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/WIFI.html", "text/html");

  });
  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler(
    "/save-config", [](AsyncWebServerRequest *request, JsonVariant &json) {
      JsonObject jsonObj = json.as<JsonObject>();
      const char* senha = jsonObj["wifiPassword"];
      ssid=jsonObj["wifiSSID"];
      senha=jsonObj["wifiPassword"];
      String mqttBrokerValue="asfjasdlfsa";
      request->send(200, "text/plain", "Mensagem recebida:");
      //deserializeJson(jsonObj,mqttBrokerValue);
      saveJson();
      setupWiFi();
      Serial.println(WiFi.localIP());
  });
  server.addHandler(handler);
  // Rota para obter configurações em JSON
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String pwm;
    String parametro;
    String sentido;
    const char *oque = "input1";
    const char *oque2 = "input2";

    if (request->hasParam(oque)) {
      pwm = request->getParam(oque)->value();
      Serial.println(pwm);
      aciona_motor(1, pwm.toInt(), 0);
    
      request->send(LittleFS, "/index.html", "text/html");
    } else if (request->hasParam(oque2)) {
      sentido = request->getParam(oque2)->value();
      Serial.println(sentido);
      sentido = sentido.toInt();
      aciona_motor(1, pwm.toInt(), sentido.toInt());
      request->send(LittleFS, "/index.html", "text/html");
    }
  });

  // Rota para obter configurações em JSON
  server.on("/config.json", HTTP_GET, handleConfigJson);
  // Rota para escanear redes Wi-Fi disponíveis
  server.on("/scan", HTTP_GET, scanWiFiNetworks);
  // Rota para verificar o status do Wi-Fi
  server.on("/wifi-status", HTTP_GET, handleWiFiStatus);
}

//Configuração do wifi
void setupWiFi() {
  static unsigned long lastConnectionAttempt = 0;
  const unsigned long connectionInterval = 10000; // Intervalo de 5 segundos entre as tentativas de conexão
  
  if (!conectado) {
    //WiFi.mode();
    WiFi.softAP("PDCA " + WiFi.macAddress(),"manut_aut");
    //Serial.print("Conectando ao WiFi");
    if (WiFi.status() != WL_CONNECTED && millis() - lastConnectionAttempt > connectionInterval) {
      WiFi.begin(ssid, senha);
      lastConnectionAttempt = millis(); // Atualiza o tempo da última tentativa de conexão
    }
  } else {
    WiFi.disconnect();
    WiFi.begin(ssid, senha);
  }

  // Configura um temporizador para verificar a conexão WiFi após 1 segundo
  static unsigned long wifiCheckTimer = millis() + 1000;

  // Verifica a conexão WiFi de forma assíncrona após o temporizador expirar
  if (millis() >= wifiCheckTimer) {
    if (WiFi.status() == WL_CONNECTED) {
      //conectado = true;
      //Serial.println("");
      //Serial.println("WiFi conectado");
      //Serial.print("Endereço IP: ");
      //Serial.println(WiFi.localIP());
    } else {
      //Serial.print(".");
    }
    wifiCheckTimer = millis() + 1000; // Reinicia o temporizador para a próxima verificação
  }
}

//Escaneia as redes WiFi disponíveis
void scanWiFiNetworks(AsyncWebServerRequest *request) {
  int n=WiFi.scanComplete();
  if(n== -2){
    WiFi.scanNetworks(true);
    request->send(200, "text/plain", "nada");  
  }
  else if(n) {
    String networks = "";
    for (int i = 0; i < n; ++i) {
      networks += WiFi.SSID(i) + "\n";
    }
    Serial.println(networks);
    WiFi.scanDelete();
    if(WiFi.scanComplete()==-2){
      WiFi.scanNetworks(true);
    }    
    request->send(200, "text/plain", networks);
  }
  else{
    request->send(200, "text/plain", "nada");
  } 
  
}

//Verifica o status do WiFi
void handleWiFiStatus(AsyncWebServerRequest *request) {
  if (WiFi.status() == WL_CONNECTED) {
    String message = "<p align='center' style='color: green;'>Conectado ao Wi-Fi: " + String(WiFi.SSID()+"</p>");
    request->send(200, "text/html", message);
  } else {
    request->send(200, "text/html", "<p align='center' style='color: red;'>Não conectado ao Wi-Fi.</p>");
  }
}
