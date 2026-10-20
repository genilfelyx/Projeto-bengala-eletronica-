
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>

// Sensores ultrassonicos
const int trig1 = 7;
const int echo1 = 6;

const int trig2 = 4;
const int echo2 = 5;

const int trig3 = 2;
const int echo3 = 3;

// Limites de deteccao
const float distanciaChao = 30.0;
const float diferencaBuraco = 15.0;
const float limiteObstaculo = 40.0;

// MPU6050
const int enderecoMPU = 0x68;
const float anguloMinBengala = 10.0;
const float anguloMaxBengala = 60.0;
bool mpuConectado = false;

// Buzzer, chave e servo
const int buzzer = 8;
const int botao = A3;
const int pinoServo = 9;

const int servPause = 90;
const int servClique = 65;

Servo servo;

// Controle da exibicao serial
int ultimoAlerta = -1;
bool ultimoEstadoBuraco = false;
unsigned long ultimaImpressao = 0;
const unsigned long intervaloSerial = 500;

// Inicializa o MPU6050
void iniciarMPU() {
  Wire.begin();

  Wire.beginTransmission(enderecoMPU);
  Wire.write(0x6B);
  Wire.write(0x00);

  mpuConectado = (Wire.endTransmission() == 0);
}

// Le os dados do acelerometro
bool lerMPU(float &ax, float &ay, float &az) {
  if (!mpuConectado) {
    return false;
  }

  Wire.beginTransmission(enderecoMPU);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(enderecoMPU, 6, true);

  if (Wire.available() < 6) {
    return false;
  }

  int16_t rawAX = Wire.read() << 8 | Wire.read();
  int16_t rawAY = Wire.read() << 8 | Wire.read();
  int16_t rawAZ = Wire.read() << 8 | Wire.read();

  ax = rawAX / 16384.0;
  ay = rawAY / 16384.0;
  az = rawAZ / 16384.0;

  return true;
}

// Verifica se o sistema esta desligado
bool sistemaDesligado() {
  return digitalRead(botao) == HIGH;
}

// Mede a distancia de um sensor ultrassonico
float medirDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long duracao = pulseIn(echo, HIGH, 30000);

  if (duracao == 0) {
    return -1;
  }

  return duracao * 0.0343 / 2.0;
}

// Aciona o buzzer e o servo
void tocarAlerta(int frequencia, int duracao, int pausa) {
  if (sistemaDesligado()) {
    return;
  }

  servo.write(servClique);
  tone(buzzer, frequencia);
  delay(duracao);
  noTone(buzzer);
  servo.write(servPause);
  delay(pausa);
}

// Executa os sons de alerta
void alertaBuzzer(bool buraco) {
  if (buraco) {
    tocarAlerta(2500, 250, 100);
    tocarAlerta(2000, 250, 100);
  } else {
    tocarAlerta(2000, 200, 80);
    tocarAlerta(2500, 200, 80);
  }
}

// Coloca os componentes em estado de repouso
void desligarSistema() {
  noTone(buzzer);

  digitalWrite(trig1, LOW);
  digitalWrite(trig2, LOW);
  digitalWrite(trig3, LOW);

  servo.write(servPause);
}

// Inicializa os sensores ultrassonicos
void iniciarSensores() {
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);

  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
}

// Inicializa buzzer e chave
void iniciarBuzzerEBotao() {
  pinMode(buzzer, OUTPUT);
  pinMode(botao, INPUT_PULLUP);
}

// Inicializa o servo motor
void iniciarServo() {
  servo.attach(pinoServo);
  servo.write(servPause);
}

// Detecta obstaculos ou possivel buraco
int detectarAlerta(
  float distancia1,
  float distancia2,
  float distancia3,
  bool leituraMPUValida,
  bool inclinacaoAdequada,
  bool &buraco
) {
  buraco = false;

  if (
    distancia1 >= 0 &&
    distancia1 > distanciaChao + diferencaBuraco &&
    leituraMPUValida &&
    inclinacaoAdequada
  ) {
    buraco = true;
    return 1;
  }

  if (distancia1 >= 0 && distancia1 < limiteObstaculo) {
    return 1;
  }

  if (distancia2 >= 0 && distancia2 < limiteObstaculo) {
    return 2;
  }

  if (distancia3 >= 0 && distancia3 < limiteObstaculo) {
    return 3;
  }

  return 0;
}

void setup() {
  Serial.begin(9600);

  iniciarSensores();
  iniciarBuzzerEBotao();
  iniciarServo();
  iniciarMPU();

  Serial.println("=== BENGALA ELETRONICA ===");

  if (mpuConectado) {
    Serial.println("MPU6050: conectado");
  } else {
    Serial.println("ERRO: MPU6050 nao encontrado!");
  }

  Serial.println("Sensores ultrassonicos: iniciados");
  Serial.println("Servo motor: iniciado");
  Serial.println("Sistema iniciado.");
  Serial.println("--------------------------");
}

void loop() {
  if (sistemaDesligado()) {
    desligarSistema();

    if (ultimoAlerta != -1) {
      Serial.println("Sistema desligado.");
      ultimoAlerta = -1;
      ultimoEstadoBuraco = false;
    }

    delay(100);
    return;
  }

  float distancia1 = medirDistancia(trig1, echo1);
  delay(50);

  float distancia2 = medirDistancia(trig2, echo2);
  delay(50);

  float distancia3 = medirDistancia(trig3, echo3);
  delay(50);

  float ax = 0;
  float ay = 0;
  float az = 0;

  float inclinacaoX = 0;
  float inclinacaoY = 0;

  bool leituraMPUValida = lerMPU(ax, ay, az);
  bool inclinacaoAdequada = false;

  if (leituraMPUValida) {
    inclinacaoX = atan2(ay, az) * 180.0 / PI;

    inclinacaoY =
      atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    inclinacaoAdequada =
      fabs(inclinacaoY) >= anguloMinBengala &&
      fabs(inclinacaoY) <= anguloMaxBengala;
  }

  // Distancias e inclinacao: exibidas a cada 500 ms
  if (millis() - ultimaImpressao >= intervaloSerial) {
    ultimaImpressao = millis();

    Serial.print("S1: ");
    if (distancia1 < 0) {
      Serial.print("--");
    } else {
      Serial.print(distancia1, 1);
    }

    Serial.print(" cm | S2: ");
    if (distancia2 < 0) {
      Serial.print("--");
    } else {
      Serial.print(distancia2, 1);
    }

    Serial.print(" cm | S3: ");
    if (distancia3 < 0) {
      Serial.print("--");
    } else {
      Serial.print(distancia3, 1);
    }

    Serial.print(" cm | Inclinacao: ");
    Serial.print(inclinacaoY, 1);
    Serial.println(" graus");
  }

  bool buraco = false;

  int sensorAlerta = detectarAlerta(
    distancia1,
    distancia2,
    distancia3,
    leituraMPUValida,
    inclinacaoAdequada,
    buraco
  );

  // Exibe uma mensagem quando o estado do alerta muda
  bool estadoMudou =
    sensorAlerta != ultimoAlerta ||
    (sensorAlerta != 0 && buraco != ultimoEstadoBuraco);

  if (estadoMudou) {
    if (sensorAlerta == 0) {
      Serial.println("Area livre.");
    } else if (buraco) {
      Serial.println("ALERTA: Possivel buraco!");
      Serial.print("Sensor: 1 | Distancia: ");
      Serial.print(distancia1, 1);
      Serial.println(" cm");

      Serial.print("Inclinacao: ");
      Serial.print(inclinacaoY, 1);
      Serial.println(" graus");
    } else {
      Serial.print("ALERTA: Obstaculo proximo!");
      Serial.print(" Sensor: ");
      Serial.print(sensorAlerta);
      Serial.print(" | Distancia: ");

      float distanciaAlerta = distancia1;

      if (sensorAlerta == 2) {
        distanciaAlerta = distancia2;
      } else if (sensorAlerta == 3) {
        distanciaAlerta = distancia3;
      }

      Serial.print(distanciaAlerta, 1);
      Serial.println(" cm");
    }

    ultimoAlerta = sensorAlerta;
    ultimoEstadoBuraco = buraco;
  }

  if (sensorAlerta != 0) {
    alertaBuzzer(buraco);

    if (sistemaDesligado()) {
      desligarSistema();
      ultimoAlerta = -1;
      ultimoEstadoBuraco = false;
    }
  } else {
    noTone(buzzer);
    servo.write(servPause);
  }

  delay(20);
}
