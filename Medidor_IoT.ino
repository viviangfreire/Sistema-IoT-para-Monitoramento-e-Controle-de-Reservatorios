#include <OneWire.h>
#define Offset 20.75

const int TRIG_US = 14;            // Pino D4 conectado ao TRIG do sensor de ultrassom
const int ECHO_US = 12;            // Pino D2 conectado ao ECHO do sensor de ultrassom
const int LED_DIST = 5;           // Pino D5 conectado ao LED referente à distância
const int SENSOR_TEMP = 21;       // Pino D13 conectado ao SENSOR referente à temperatura
const int LED_TEMP = 19;          // Pino D18 conectado ao LED referente à temperatura
const int POT_OXIG = 34;          // Pino D32 conectado ao POTENCIÔMETRO referente à oxigenação
const int LED_OXIG = 18;          // Pino D19 conectado ao LED referente à oxigenação
const int SENSOR_PH = 32;         // Pino D36 conectado ao SENSOR referente ao pH
const int PIN_RELE = 15;          // Pino D36 conectado ao POTENCIÔMETRO referente ao pH 

OneWire oneWire(SENSOR_TEMP);     //Criação do objeto oneWire que irá controlar o barramento de comunicação com o sensor

void setup() {
  Serial.begin(9600);               // Inicializa a comunicação serial
  pinMode(TRIG_US, OUTPUT);         // Configura o pino TRIG como saída
  pinMode(ECHO_US, INPUT);          // Configura o pino ECHO como entrada
  pinMode(LED_DIST, OUTPUT);        // Configura o LED (referente à distância) como saída
  pinMode(LED_TEMP, OUTPUT);        // Configura o LED (referente à temperatura) como saída
  pinMode(SENSOR_PH, INPUT);        // Configura o POTENCIÔMETRO (referente ao pH) como entrada
  pinMode(POT_OXIG, INPUT);         // Configura o POTENCIÔMETRO (referente à oxigenação) como entrada
  pinMode(LED_OXIG, OUTPUT);        // Configura o LED (referente à oxigenação) como saída
  pinMode(PIN_RELE, OUTPUT);
}

void loop() {
  //-------------------- MEDIÇÃO DA DISTÂNCIA -------------------------
  digitalWrite(TRIG_US, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_US, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_US, LOW);
  long duracao = pulseIn(ECHO_US, HIGH);        // Mede o tempo de resposta do ECHO  
  float distancia = (duracao * 0.0343) / 2;     // Calcula a distância usando a velocidade do som (aproximadamente 343 m/s)
  Serial.print("Distância: ");
  Serial.print(distancia);
  Serial.println(" cm");
  digitalWrite(LED_DIST, distancia >= 20 ? HIGH : LOW);
  

  //------------------- MEDIÇÃO DA TEMPERATURA -------------------------
  float tempC = readTemperature();             // Função para ler a temperatura
  Serial.print("Temperatura: ");
  Serial.print(tempC);
  Serial.println(" °C");
  digitalWrite(LED_TEMP, tempC <= 20 ? HIGH : LOW);


  //------------------------ MEDIÇÃO DO PH -------------------------------
  int sensorValue = analogRead(SENSOR_PH);     // Lê o valor analógico
  float Voltage, pHValue;
  Voltage = sensorValue * (3.3 / 4095.0);      // Converte para tensão (3.3V para ESP32)
  pHValue = -15.9 * Voltage + Offset;            // Fórmula para cálculo de pH
  Serial.print("pH: ");
  Serial.println(pHValue);


  //------------------------ MEDIÇÃO DA OXIGENAÇÃO ----------------------
  float oxigenacao = analogRead(POT_OXIG)/292.5;
  Serial.print("Oxigenação: ");
  Serial.print(oxigenacao);
  Serial.println(" mg/L");
  digitalWrite(LED_OXIG, oxigenacao <= 4 ? HIGH : LOW);


  //------------------------ RELÉ ---------------------------------------
  if(distancia >= 15){
    digitalWrite(PIN_RELE, HIGH);
  }else if(distancia <= 10){
    digitalWrite(PIN_RELE, LOW);
  }

  Serial.println("-------------------------"); // Fim da leitura
  delay(500);                                 //Aguardando 2 segundos para iniciar a próxima leitura

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