/* 
  nas funções distanceSensor() e temperatureSensor(),
  foi comentada as linhas para printar seus respectivos valores no terminal
  para testar a integração dos valores no App.

  é necessário testar o código abaixo com todos os sensores para confirmação da integração.
*/

#include <OneWire.h>
#include <WiFiManager.h> 
#include <WiFi.h>
#include <PubSubClient.h>

#define Led  32
#define Offset 20.75
//MQTT SETUP
#define MQTT_ID "levirabelo-esp32-medidoriot32-ifce/llv"
#define MQTT_BROKER "broker.emqx.io"
//#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TERMINAL_TOPIC "ident-levir_esp32_medidoriotifce/terminal"
#define MQTT_SECONDS_TOPIC "ident-levir_esp32_medidoriotifces"
#define MQTT_MINUTES_TOPIC "ident-levir_esp32_medidoriotifcem"
#define MQTT_HOURS_TOPIC "ident-levir_esp32_medidoriotifceh"
#define MQTT_DAYS_TOPIC "ident-levir_esp32_medidoriotifced"
#define MQTT_LED_TOPIC "ident-levir_esp32_medidoriotifce/led"
#define MQTT_DISTANCE_TOPIC "ident-levir_esp32_medidoriotifce/distance"
#define MQTT_VOLUME_TOPIC "ident-levir_esp32_medidoriotifce/volume"
#define MQTT_TEMPERATURE_TOPIC "ident-levir_esp32_medidoriotifce/temperature"
#define MQTT_PH_TOPIC "ident-levir_esp32_medidoriotifce/ph"
#define MQTT_OXIG_TOPIC "ident-levir_esp32_medidoriotifce/oxg"

WiFiClient espClient; // cliente de rede
PubSubClient MQTT(espClient); // cliente MQTT

char seconds_str[4] = "";
char minutes_str[4] = "";
char hours_str[3] = "";
char days_str[5] = "";
char dist_str[6] = "";
char volume_str[7] = "";
char temp_str[4] = "";
char ph_str[4] = "";
char oxg_str[6] = "";

// Configurações e Endereço de Internet WIFI
// WiFiServer server(8090);
IPAddress ip;
char ipChar[27];
/*Variável para armazenar a solicitação HTTP
String header;
// Hora Atual
unsigned long currentTime = millis();
// Vez anterior
unsigned long previousTime = 0; 
//Defina o tempo limite em milissegundos 
const long timeoutTime = 2000;*/

// Reset Wifi Manager
WiFiManager wifiManager;

// Definição de variáveis auxiliares de tempo para mostrar no dashboard
int aux = 1, aSeg = 0, aMin = 0, aH = 0;
int sec = 0, secondsAux = 60;
int MIN = 0, minutesAux = 60;
int h = 0, hoursAux = 24;
int d = 0, daysAux = 1;

// Definição de variáveis auxiliares de controle do Led
int ledAux = 1, ledAuxCallback = 3;

void publishTimeDashboard() {
  sprintf(days_str, "%d", d);
  MQTT.publish(MQTT_DAYS_TOPIC, days_str);
  sprintf(hours_str, "%d", aH);
  MQTT.publish(MQTT_HOURS_TOPIC, hours_str);
  sprintf(minutes_str, "%d", aMin);
  MQTT.publish(MQTT_MINUTES_TOPIC, minutes_str);
  sprintf(seconds_str, "%d", aSeg);
  MQTT.publish(MQTT_SECONDS_TOPIC, seconds_str);
}

// Função para mostrar o tempo de atividade no Dashboard do broker na nuvem
void timeDashboard() {
  /*
    A cada ciclo de tempo em segundos, os minutos são acrescentados e segundos resetados.
    A cada ciclo de tempo em minutos, as horas são acrescentadas e minutos e segundos resetados.
    E assim por diante...
    Os dias serão apenas acrescentados.
  */
  if (millis() - sec >= 1000) {
    sec = millis();
    aSeg++;
    publishTimeDashboard();
    if (millis()/1000 >= secondsAux) {
      MIN = millis();
      aMin++;
      publishTimeDashboard();
      secondsAux = secondsAux+60;
      aSeg = 0;
      if (millis()/60000 >= minutesAux) {
        h = millis();
        aH++;
        publishTimeDashboard();
        minutesAux = minutesAux+60;
        aMin = 0;
        if(millis()/3600000 >= hoursAux) {
          daysAux = daysAux+1;
          d++;
          publishTimeDashboard();
          hoursAux = hoursAux+24;
          aH = 0;
        }
      }
    }
  }
  publishTimeDashboard();
}

// Função para conectar/reconectar ao wifi caso não esteja conectado ou caso a conexão caia
void connect_wifi() {
  if (aux == 1 && WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("------------------------------------------------------------");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("------------------------------------------------------------");
    ip = WiFi.localIP();
    snprintf(ipChar, sizeof(ipChar), "IP Local: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    digitalWrite(2, HIGH); // Controle do led azul em caso de conexão com wifi seja bem sucedida
    delay(200);
    digitalWrite(2, LOW);
    delay(350);
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(350);
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(350);

    // server.begin();
    aux++;
  }
  else {
    while (WiFi.status() != WL_CONNECTED) {
      aux = 1;
      Serial.println();
      Serial.println();
      Serial.print("Connecting...");

      for (int i=1; i<= 12; i++) {
        delay(500);
        Serial.println(".");
        if (i == 1 && WiFi.status() != WL_CONNECTED) { // Controle do led azul caso a conexão não esteja estabelecida
          digitalWrite(2, HIGH);
        }
        if (i == 4 && WiFi.status() != WL_CONNECTED) {
          digitalWrite(2, LOW);
        }
        if (i == 12 && WiFi.status() != WL_CONNECTED) {i=0;}

        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("");
          Serial.println("------------------------------------------------------------");
          Serial.println("WiFi connected.");
          Serial.println("IP address: ");
          Serial.println(WiFi.localIP());
          Serial.println("------------------------------------------------------------");
          ip = WiFi.localIP();
          snprintf(ipChar, sizeof(ipChar), "IP Local: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

          digitalWrite(2, HIGH);
          delay(200);
          digitalWrite(2, LOW);
          delay(350);
          digitalWrite(2, HIGH);
          delay(200);
          digitalWrite(2, LOW);
          delay(350);
          digitalWrite(2, HIGH);
          delay(200);
          digitalWrite(2, LOW);
          delay(350);

          // server.begin();
          aux++;
          break;
        }
      }
      break;
    }
  }
}

// função para a inicialização e conexão do mqtt na nuvem
void setupMQTT() {
  while (!MQTT.connected())
  /*
    Lembrando que se o Wi-Fi em que o ESP estiver conectado não possuir conexão com a internet, o mesmo não terá acesso a nuvem.
    Se isso acontecer, a função entrará em loop até que a conexão seja estabelecida.
  */
  {
    digitalWrite(2, LOW);
    Serial.print("*Tentando se conectar ao Broker MQTT: ");
    Serial.println(MQTT_BROKER);
    Serial.print(".");
    digitalWrite(2, HIGH); // Controle de leds caso a conexão não esteja estabelecida
    delay(500);
    digitalWrite(2, LOW);
    delay(50);

    if(MQTT.connect(MQTT_ID)) // Caso a conexão seja re/estabelecida
    {
      Serial.println("Conectado ao broker MQTT com sucesso.");
      for (int k=1; k<=8; k++) { // Controle de leds caso a conexão seja re/estabelecida
        digitalWrite(Led, !digitalRead(Led));
        delay(150);
        if (k==4 || k==8) {
          delay(450);
        }
      }
      // .subscribe(topic); é a função em que o tópico selecionado deseja receber informações (payload) do aplicativo (callback)
      MQTT.subscribe("ident-levir_esp32_medidoriotifce/led");
      MQTT.subscribe("ident-levir_esp32_medidoriotifce/distances");
      MQTT.subscribe("ident-levir_esp32_medidoriotifce/temperatures");
      MQTT.subscribe("ident-levir_esp32_medidoriotifce/phs");
      MQTT.subscribe("ident-levir_esp32_medidoriotifce/oxgs");
      MQTT.subscribe("ident-levir_esp32_medidoriotifce/waterpump");
      MQTT.publish(MQTT_TERMINAL_TOPIC, "Conectado ao servidor.");
    }
    else
    {
      Serial.println("Falha ao reconectar ao broker.");
      Serial.println("Havera nova tentativa de conexao em 5s");
      delay(5000);
    }
  }
}

// Configuração dos sensores

const int TRIG_US = 4;            // Pino D4 conectado ao TRIG do sensor de ultrassom
const int ECHO_US = 2;            // Pino D2 conectado ao ECHO do sensor de ultrassom
const int LED_DIST = 5;           // Pino D5 conectado ao LED referente à distância
const int SENSOR_TEMP = 19;       // Pino D19 conectado ao SENSOR referente à temperatura
const int LED_TEMP = 18;          // Pino D18 conectado ao LED referente à temperatura
const int POT_OXIG = 36;          // Pino D36 conectado ao POTENCIÔMETRO referente à oxigenação
const int LED_OXIG = 21;          // Pino D21 conectado ao LED referente à oxigenação
const int SENSOR_PH = 34;         // Pino D34 conectado ao SENSOR referente ao pH
const int PIN_RELE = 15;          // Pino D15 conectado ao POTENCIÔMETRO referente ao pH 

OneWire oneWire(SENSOR_TEMP);     //Criação do objeto oneWire que irá controlar o barramento de comunicação com o sensor

// Definição de variáveis auxiliares de distancia, temperatura e ph para mostrar no dashboard
int aux2SecondsDelay = 1, distCallback = (1-2), tempCallback, phCallback, oxgCallback, auxTerminal = 1, waterPumpAuxCallback = 0;
float tempC, distancia, disgraph, volume, pH, oxG;

String waterPumpCallback;

void setup() {
  Serial.begin(115200);
  delay(50);
  pinMode(TRIG_US, OUTPUT);         // Configura o pino TRIG como saída
  pinMode(ECHO_US, INPUT);          // Configura o pino ECHO como entrada
  pinMode(LED_DIST, OUTPUT);        // Configura o LED (referente à distância) como saída
  pinMode(LED_TEMP, OUTPUT);        // Configura o LED (referente à temperatura) como saída
  pinMode(SENSOR_PH, INPUT);        // Configura o POTENCIÔMETRO (referente ao pH) como entrada
  pinMode(POT_OXIG, INPUT);         // Configura o POTENCIÔMETRO (referente à oxigenação) como entrada
  pinMode(LED_OXIG, OUTPUT);        // Configura o LED (referente à oxigenação) como saída
  pinMode(PIN_RELE, OUTPUT);        // Configura o Módulo Relé como saída

  Serial.println("Configurar Rede Wifi");

  //  SETUP -- wifi -- ////////////////////////////
  /*
    Caso o ESP não tenha uma conexão de Wi-Fi salva para se conectar,
    ele vira um AP (Access Point) definido pelo desenvolvedor com SSID e senha, em que a gente possa se conectar a ele. gateway: 192.168.4.1
    A partir disso, poderemos configurar uma rede Wi-Fi para que ele possa se conectar.
  */
  wifiManager.autoConnect("MedidorIoT", "23456789"); // (SSID, senha)

  delay(150);
  WiFi.begin();

  connect_wifi();

  MQTT.setServer(MQTT_BROKER, MQTT_PORT); // Servidor em que o ESP se conectará
  MQTT.setCallback(callback); // receber informações do aplicativo e executá-las 

  delay(100);
  
  Serial.print("");
  Serial.println("INICIOU **************************");
}

void ledConnected() { // função para interação do Led azul do esp com o aplicativo
  if (ledAuxCallback == 0) {digitalWrite(Led, LOW);}
  else if (ledAuxCallback == 1) {digitalWrite(Led, HIGH);}
  else if (ledAuxCallback == 2 || ledAuxCallback == 3){
    if (ledAux == 1) {
      digitalWrite(Led, HIGH);
    } else if (ledAux == 3) {
      digitalWrite(Led, LOW);
    }
    if (ledAux == 12) {
      ledAux = 0;
    }
    ledAux++;
  }
}

// callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageLed;
  char terminalMessage[50];
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageLed += (char)payload[i];
  }
  Serial.println();

  if (String(topic) == "ident-levir_esp32_medidoriotifce/led") { // função callback para ligar/desligar led pelo aplicativo
    Serial.print("Changing output to ");
    if(messageLed == "on"){
      Serial.println("on");
      digitalWrite(Led, HIGH);
      ledAuxCallback = 1;
      MQTT.publish(MQTT_TERMINAL_TOPIC, "Mensagem para acender o Led azul D2 do ESP.");
    }
    else if(messageLed == "off"){
      Serial.println("off");
      digitalWrite(Led, LOW);
      ledAuxCallback = 0;
      MQTT.publish(MQTT_TERMINAL_TOPIC, "Mensagem para apagar o Led azul D2 do ESP.");
    }
    else if (messageLed == "lednormalconnection") {
      Serial.println("normal led indicator connection");
      ledAuxCallback = 2;
      MQTT.publish(MQTT_TERMINAL_TOPIC, "Mensagem para deixar o Led azul D2 do ESP com indicação de conexão.");
    }
  }

  if (String(topic) == "ident-levir_esp32_medidoriotifce/distances") { // função callback para determinar a distância em que o Led da distância ficará ligado
    payload[length] = '\0';
    distCallback = atoi((char*)payload);
    snprintf(terminalMessage, sizeof(terminalMessage), "Profundidade limite definida para %d cm", distCallback);
    MQTT.publish(MQTT_TERMINAL_TOPIC, terminalMessage);
  }

  if (String(topic) == "ident-levir_esp32_medidoriotifce/temperatures") { // função callback para determinar a temperatura em que o Led da temperatura ficará ligado
    payload[length] = '\0';
    tempCallback = atof((char*)payload);
    int tempInt = tempCallback * 100;
    snprintf(terminalMessage, sizeof(terminalMessage), "Temperatura limite definida para %d ºC", tempInt/100);
    MQTT.publish(MQTT_TERMINAL_TOPIC, terminalMessage);
  }

  if (String(topic) == "ident-levir_esp32_medidoriotifce/phs") {
    payload[length] = '\0';
    phCallback = atof((char*)payload);
    int phInt = phCallback * 100;
    snprintf(terminalMessage, sizeof(terminalMessage), "pH limite definido para %d", phInt/100);
    MQTT.publish(MQTT_TERMINAL_TOPIC, terminalMessage);
  }

  if (String(topic) == "ident-levir_esp32_medidoriotifce/oxgs") {
    payload[length] = '\0';
    oxgCallback = atof((char*)payload);
    int oxgInt = oxgCallback * 100;
    snprintf(terminalMessage, sizeof(terminalMessage), "Oxigenação limite definida para %d", oxgInt/100);
    MQTT.publish(MQTT_TERMINAL_TOPIC, terminalMessage);
  }

  if (String(topic) == "ident-levir_esp32_medidoriotifce/waterpump") {

    for (int i = 0; i < length; i++) {
      waterPumpCallback += (char)payload[i];
    }

    if (waterPumpCallback == "auto") {
      if (distCallback == (1-2)) { //  distCallback == (1-2) -> quando a profundidade limite não for definida
        MQTT.publish(MQTT_TERMINAL_TOPIC, "Defina a profundidade limite antes de utilizar a configuração automática da bomba.");
      }
      else { 
        // função para ativar a bomba no modo automático
        if (distCallback <= distancia) {
          // ligar a bomba
          digitalWrite(PIN_RELE, LOW);
          Serial.println("Ligado");
        }
        else {
          // desligar a bomba
          digitalWrite(PIN_RELE, HIGH);
          Serial.println("Desligado");
      }
    }
  }
    if (waterPumpCallback == "manual") {
      MQTT.publish(MQTT_TERMINAL_TOPIC, "Bomba d'água configurada para modo manual.");
      waterPumpAuxCallback = 2;
      digitalWrite(PIN_RELE, LOW);
    }
    if (waterPumpCallback == "on") {
      if (waterPumpAuxCallback == 0 || waterPumpAuxCallback == 1) {
        MQTT.publish(MQTT_TERMINAL_TOPIC, "Defina a configuração da bomba para manual.");
      }
      else if (waterPumpAuxCallback = 2) {
        MQTT.publish(MQTT_TERMINAL_TOPIC, "Bomba d'água ligada.");
        digitalWrite(PIN_RELE, HIGH);
        waterPumpAuxCallback = 3;
      }
    }
    else if (waterPumpCallback == "off") {
      if (waterPumpAuxCallback == 0 || waterPumpAuxCallback == 1) {
        MQTT.publish(MQTT_TERMINAL_TOPIC, "Defina a configuração da bomba para manual.");
      }
      else if (waterPumpAuxCallback = 3) {
        MQTT.publish(MQTT_TERMINAL_TOPIC, "Bomba d'água desligada.");
        digitalWrite(PIN_RELE, LOW);
        waterPumpAuxCallback = 2;
      }
    }
  }
  waterPumpCallback = "";

}

void distanceSensor() {
  timeDashboard();
  digitalWrite(TRIG_US, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_US, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_US, LOW);
  long duracao = pulseIn(ECHO_US, HIGH);        // Mede o tempo de resposta do ECHO  
  distancia = 27 - ((duracao * 0.0343) / 2);     // Calcula a distância usando a velocidade do som (aproximadamente 343 m/s)
  
  if(distancia >= distCallback){
    digitalWrite(LED_DIST, LOW); // Se a distância for maior que a distância determinda pelo usuário do app, LED apagado
  }else{
    digitalWrite(LED_DIST, HIGH); // Caso não, LED aceso
  }

  sprintf(dist_str, "%.2f", distancia);
  MQTT.publish(MQTT_DISTANCE_TOPIC, dist_str);
  delay(20);

  Serial.println(".");
  Serial.println(distancia);
}

void tempratureSensor() {
  timeDashboard();
  tempC = readTemperature();             // Função para ler a temperatura

  if(tempC < tempCallback){
    digitalWrite(LED_TEMP, HIGH);        // Se a temperatura for menor que a temperatura determinda pelo usuário do app, LED aceso
  }else{
    digitalWrite(LED_TEMP, LOW);         // Caso não, LED apagado
  }

  sprintf(temp_str, "%.2f", tempC);
  MQTT.publish(MQTT_TEMPERATURE_TOPIC, temp_str);
  delay(20);

  Serial.println(tempC);
}

void phSensor() {
  timeDashboard();
  int sensorValue = analogRead(SENSOR_PH);     // Lê o valor analógico
  float Voltage;

  Voltage = sensorValue * (3.3 / 4095.0);      // Converte para tensão (3.3V para ESP32)
  pH = -15.9 * Voltage + Offset;            // Fórmula para cálculo de pH

  if (pH < phCallback) {
    // led do ph aceso
  } else {
    // led do ph desligado
  }

  sprintf(ph_str, "%.2f", pH);
  MQTT.publish(MQTT_PH_TOPIC, ph_str);
  delay(20);

  Serial.println(pH);
  Serial.println("Voltagem: ");
  Serial.println(Voltage);
}

void oxgSensor() {
  // função de configuração do sensor de Oxigenação (de imediato: potenciômetro)
  timeDashboard();
  oxG = analogRead(POT_OXIG)/292.5;

  if (oxG <= oxgCallback) {
    // led da oxg aceso
    digitalWrite(LED_OXIG, HIGH);
  } else {
    // led da oxg apagado
    digitalWrite(LED_OXIG, LOW);
  }

  sprintf(oxg_str, "%.2f", oxG);
  MQTT.publish(MQTT_OXIG_TOPIC, oxg_str);
  delay(20);

  Serial.println(oxG);
  Serial.println(".");
}

void WaterPumpOnOff() {
  int WPAux;
  timeDashboard();

  if (waterPumpCallback == "auto") {
    // desligar a bomba primeiro e depois configurar para funcionar no modo automático
    if (distCallback != (1-2)) { /*  distCallback == (1-2) -> quando a profundidade limite não for definida */
      // função para ativar a bomba no modo automático
      if (distCallback <= distancia) {
        // ligar a bomba
        digitalWrite(PIN_RELE, LOW);
        Serial.println("Ligado");
      }
      else {
        // desligar a bomba
        digitalWrite(PIN_RELE, HIGH);
        Serial.println("Desligado");
      }
    }

    else {digitalWrite(PIN_RELE, LOW);}
  }
  
  else if (waterPumpCallback == "manual") {
    // desligar a bomba primeiro e depois configurar para funcionar no modo manual
    if (WPAux == 1) {
      digitalWrite(PIN_RELE, LOW);
      WPAux = 0;
    }
    
    if (waterPumpAuxCallback = 3) { // Quando waterPumpAuxCallback = 3, a bomba está manualmente ligada
      // função para ativar a bomba no modo manual
      digitalWrite(PIN_RELE, HIGH);
      Serial.println("Ligado");
    }
    else if (waterPumpAuxCallback = 2)  { // Quando waterPumpAuxCallback = 2, a bomba está manualmente desligada
      // função para desativar a bomba no modo manual
      digitalWrite(PIN_RELE, LOW);
      Serial.println("Desligado");
    }
  }

  delay(50);
}

void loop() {
  // chama as funções para verificação de conexão e enviar/receber informações para o aplicativo
  connect_wifi();
  if (!MQTT.connected()) {
    setupMQTT();
  }

  // Caso o ESP32 reinicie, mostrar que os valores não foram definidos
  if (auxTerminal == 1) {
    MQTT.publish(MQTT_TERMINAL_TOPIC, ipChar);
    MQTT.publish(MQTT_TERMINAL_TOPIC, "Profundidade limite não definida.");
    MQTT.publish(MQTT_TERMINAL_TOPIC, "Temperatura limite não definida.");
    MQTT.publish(MQTT_TERMINAL_TOPIC, "pH limite não definido.");
    MQTT.publish(MQTT_TERMINAL_TOPIC, "Oxigenação limite não definida.");
    MQTT.publish(MQTT_TERMINAL_TOPIC, "Bomba d'água não configurada.");
    auxTerminal++;
  }

  distanceSensor();
  tempratureSensor();
  phSensor();
  oxgSensor();
  WaterPumpOnOff();
  ledConnected();
  timeDashboard();
  MQTT.loop();
  delay(900);

  volume = 3.14*7*7*distancia;
  sprintf(volume_str, "%.2f", volume);
  MQTT.publish(MQTT_VOLUME_TOPIC, volume_str);

}

float readTemperature() {
  oneWire.reset();                  // Iniciando uma nova comunicação
  oneWire.write(0xCC);              // Skip ROM (caso tenha apenas um sensor)
  oneWire.write(0x44);              // Inicia a conversão da temperatura
  delay(750);                       // Tempo de conversão do sensor
    
  oneWire.reset();     
  oneWire.write(0xCC);     
  oneWire.write(0xBE);              // Comando para leitura dos dados
    
  byte LSB = oneWire.read();        //(Least Significant Byte) → Primeiro byte recebido.
  byte MSB = oneWire.read();        //(Most Significant Byte) → Segundo byte recebido.

  int16_t raw = (MSB << 8) | LSB;   //Juntando os dois bytes para formar um número de 16 bits
  return raw / 16.0;                // Converte para °C
  //Dividimos por 16.0 porque o sensor DS18B20 armazena a temperatura com 4 casas decimais (exemplo: 80.5°C é armazenado como 1288, e 1288 ÷ 16 = 80.5)
}
