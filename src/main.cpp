#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>

// Pinagem do Arduino Nano
const byte TRIG_FRENTE = 2;
const byte ECHO_FRENTE = 3;
const byte TRIG_SUPERIOR = 4;
const byte ECHO_SUPERIOR = 5;
const byte TRIG_INFERIOR = 6;
const byte ECHO_INFERIOR = 7;
const byte PINO_BOTAO = 8;
const byte PINO_SERVO = 9;
const byte PINO_BUZZER = 10;

// Limites das leituras e dos alertas, em centimetros
const float LIMITE_ALERTA = 150.0f;
const float LIMITE_MEDIO = 100.0f;
const float LIMITE_PROXIMO = 50.0f;
const float DIST_MINIMA = 2.0f;
const float DIST_MAXIMA = 350.0f;

// Periodos de atualizacao
const unsigned long INTERVALO_SENSORES_MS = 150;
const unsigned long INTERVALO_ENTRE_SENSORES_MS = 60;
const unsigned long INTERVALO_MPU_MS = 100;
const unsigned long INTERVALO_STATUS_MS = 500;
const unsigned long TEMPO_CALIBRACAO_MS = 3000;
const unsigned long INTERVALO_AMOSTRA_MS = 20;
const unsigned long DEBOUNCE_MS = 40;

// Configuracao do MPU-6050
const byte MPU_ADDR = 0x68;
bool mpuDisponivel = false;
bool calibracaoConcluida = false;
float anguloZero = 0.0f;
float anguloAtual = 0.0f;

// Distancias atuais; -1 representa uma leitura sem eco valido
float distFrente = -1.0f;
float distSuperior = -1.0f;
float distInferior = -1.0f;

unsigned long ultimoCicloSensores = 0;
unsigned long ultimaLeituraMPU = 0;
unsigned long ultimoStatus = 0;

// O botao usa INPUT_PULLUP: LOW significa pressionado
bool buzzerAtivo = true;
bool leituraAnterior = HIGH;
bool estadoEstavel = HIGH;
unsigned long ultimaMudancaBotao = 0;

// Estado do alerta sonoro intermitente
bool buzzerLigado = false;
unsigned long inicioFaseBuzzer = 0;
unsigned long duracaoFaseBuzzer = 0;

// Estado do mecanismo tatil
Servo servoTatil;
const int SERVO_REPOUSO = 90;
const int SERVO_A = 82;
const int SERVO_B = 98;
const unsigned long DURACAO_PULSO_SERVO_MS = 130;
bool servoEmPulso = false;
bool proximaPosicao = false;
unsigned long inicioPulsoServo = 0;
unsigned long ultimoPulsoServo = 0;

// Comunicacao com o MPU-6050
bool escreverMPU(byte reg, byte valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

int16_t lerInt16() {
  const uint16_t alto = static_cast<uint16_t>(Wire.read());
  const uint16_t baixo = static_cast<uint16_t>(Wire.read());
  return static_cast<int16_t>((alto << 8) | baixo);
}

bool iniciarMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I

  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(MPU_ADDR, static_cast<byte>(1)) != 1) {
    return false;
  }

  if (Wire.read() != 0x68) {
    return false;
  }

  // Acorda o sensor e seleciona a faixa de aceleracao de +/- 2 g.
  return escreverMPU(0x6B, 0x00) && escreverMPU(0x1C, 0x00);
}

bool lerAngulo(float &angulo) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Registradores iniciais do acelerometro

  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(MPU_ADDR, static_cast<byte>(6)) != 6) {
    return false;
  }

  const int16_t axRaw = lerInt16();
  const int16_t ayRaw = lerInt16();
  const int16_t azRaw = lerInt16();

  const float ax = axRaw / 16384.0f;
  const float ay = ayRaw / 16384.0f;
  const float az = azRaw / 16384.0f;

  angulo = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  return true;
}

void calibrarMPU() {
  Serial.println(F("Calibrando MPU: mantenha a bengala imovel."));

  const unsigned long inicio = millis();
  unsigned long ultimaAmostra = inicio;
  float somaAngulos = 0.0f;
  unsigned int quantidadeAmostras = 0;

  while (millis() - inicio < TEMPO_CALIBRACAO_MS) {
    const unsigned long agora = millis();

    if (agora - ultimaAmostra >= INTERVALO_AMOSTRA_MS) {
      ultimaAmostra = agora;

      float angulo;
      if (lerAngulo(angulo)) {
        somaAngulos += angulo;
        quantidadeAmostras++;
      }
    }

    delay(1);
  }

  if (quantidadeAmostras == 0) {
    Serial.println(F("ERRO: nao foi possivel calibrar o MPU."));
    return;
  }

  anguloZero = somaAngulos / quantidadeAmostras;
  calibracaoConcluida = true;

  Serial.print(F("Referencia angular: "));
  Serial.println(anguloZero, 2);
  Serial.println(F("Calibracao concluida."));
}

// Leitura dos sensores ultrassonicos
float lerDistancia(byte trig, byte echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  const unsigned long duracao = pulseIn(echo, HIGH, 24000UL);
  if (duracao == 0) {
    return -1.0f;
  }

  const float distancia = duracao * 0.0343f / 2.0f;
  if (distancia < DIST_MINIMA || distancia > DIST_MAXIMA) {
    return -1.0f;
  }

  return distancia;
}

void atualizarDistancias() {
  distFrente = lerDistancia(TRIG_FRENTE, ECHO_FRENTE);
  delay(INTERVALO_ENTRE_SENSORES_MS);

  distSuperior = lerDistancia(TRIG_SUPERIOR, ECHO_SUPERIOR);
  delay(INTERVALO_ENTRE_SENSORES_MS);

  distInferior = lerDistancia(TRIG_INFERIOR, ECHO_INFERIOR);
  ultimoCicloSensores = millis();
}

float menorDistancia() {
  float menor = -1.0f;

  if (distFrente > 0.0f && (menor < 0.0f || distFrente < menor)) {
    menor = distFrente;
  }
  if (distSuperior > 0.0f && (menor < 0.0f || distSuperior < menor)) {
    menor = distSuperior;
  }
  if (distInferior > 0.0f && (menor < 0.0f || distInferior < menor)) {
    menor = distInferior;
  }

  return menor;
}

unsigned long intervaloAlerta(float distancia) {
  if (distancia < 0.0f || distancia > LIMITE_ALERTA) {
    return 0;
  }
  if (distancia <= LIMITE_PROXIMO) {
    return 180;
  }
  if (distancia <= LIMITE_MEDIO) {
    return 350;
  }

  return 700;
}

// Controles de entrada e saida
void atualizarBotao() {
  const bool leitura = digitalRead(PINO_BOTAO);
  const unsigned long agora = millis();

  if (leitura != leituraAnterior) {
    ultimaMudancaBotao = agora;
    leituraAnterior = leitura;
  }

  if (agora - ultimaMudancaBotao < DEBOUNCE_MS || leitura == estadoEstavel) {
    return;
  }

  estadoEstavel = leitura;
  if (estadoEstavel != LOW) {
    return;
  }

  buzzerAtivo = !buzzerAtivo;
  Serial.print(F("Buzzer: "));
  Serial.println(buzzerAtivo ? F("ATIVADO") : F("DESATIVADO"));

  if (!buzzerAtivo) {
    digitalWrite(PINO_BUZZER, LOW);
    buzzerLigado = false;
  }
}

void atualizarBuzzer() {
  const unsigned long intervalo = intervaloAlerta(menorDistancia());
  const unsigned long agora = millis();

  if (!buzzerAtivo || intervalo == 0) {
    digitalWrite(PINO_BUZZER, LOW);
    buzzerLigado = false;
    return;
  }

  if (agora - inicioFaseBuzzer < duracaoFaseBuzzer) {
    return;
  }

  inicioFaseBuzzer = agora;
  buzzerLigado = !buzzerLigado;

  const unsigned long metade = intervalo / 2;
  duracaoFaseBuzzer = buzzerLigado ? metade : intervalo - metade;
  digitalWrite(PINO_BUZZER, buzzerLigado ? HIGH : LOW);
}

void atualizarServo() {
  const unsigned long intervalo = intervaloAlerta(menorDistancia());
  const unsigned long agora = millis();

  if (intervalo == 0) {
    if (servoEmPulso) {
      servoTatil.write(SERVO_REPOUSO);
      servoEmPulso = false;
    }
    return;
  }

  if (servoEmPulso &&
      agora - inicioPulsoServo >= DURACAO_PULSO_SERVO_MS) {
    servoTatil.write(SERVO_REPOUSO);
    servoEmPulso = false;
  }

  if (!servoEmPulso && agora - ultimoPulsoServo >= intervalo) {
    proximaPosicao = !proximaPosicao;
    servoTatil.write(proximaPosicao ? SERVO_A : SERVO_B);
    servoEmPulso = true;
    inicioPulsoServo = agora;
    ultimoPulsoServo = agora;
  }
}

// Exibicao de diagnostico
void imprimirDistancia(const __FlashStringHelper *nome, float distancia) {
  Serial.print(nome);
  if (distancia < 0.0f) {
    Serial.print(F("sem eco"));
  } else {
    Serial.print(distancia, 1);
    Serial.print(F(" cm"));
  }
}

void imprimirStatus() {
  imprimirDistancia(F("Frente: "), distFrente);
  Serial.print(F(" | Superior: "));
  imprimirDistancia(F(""), distSuperior);
  Serial.print(F(" | Inferior: "));
  imprimirDistancia(F(""), distInferior);

  Serial.print(F(" | Inclinacao relativa: "));
  if (mpuDisponivel && calibracaoConcluida) {
    Serial.print(anguloAtual - anguloZero, 1);
    Serial.print(F(" graus"));
  } else {
    Serial.print(F("indisponivel"));
  }

  const float distancia = menorDistancia();
  Serial.print(F(" | Alerta: "));
  if (distancia < 0.0f) {
    Serial.println(F("sem leitura valida"));
  } else if (distancia > LIMITE_ALERTA) {
    Serial.println(F("sem deteccao no limite"));
  } else if (distancia <= LIMITE_PROXIMO) {
    Serial.println(F("PROXIMO"));
  } else if (distancia <= LIMITE_MEDIO) {
    Serial.println(F("MEDIO"));
  } else {
    Serial.println(F("ATENCAO"));
  }
}

void iniciarSensores() {
  pinMode(TRIG_FRENTE, OUTPUT);
  pinMode(ECHO_FRENTE, INPUT);
  pinMode(TRIG_SUPERIOR, OUTPUT);
  pinMode(ECHO_SUPERIOR, INPUT);
  pinMode(TRIG_INFERIOR, OUTPUT);
  pinMode(ECHO_INFERIOR, INPUT);

  digitalWrite(TRIG_FRENTE, LOW);
  digitalWrite(TRIG_SUPERIOR, LOW);
  digitalWrite(TRIG_INFERIOR, LOW);
}

void setup() {
  Serial.begin(115200);

  iniciarSensores();
  pinMode(PINO_BOTAO, INPUT_PULLUP);
  pinMode(PINO_BUZZER, OUTPUT);
  digitalWrite(PINO_BUZZER, LOW);

  servoTatil.attach(PINO_SERVO);
  servoTatil.write(SERVO_REPOUSO);

  Wire.begin();
  mpuDisponivel = iniciarMPU();

  Serial.println(F("=== BENGALA ELETRONICA EXPERIMENTAL ==="));
  if (mpuDisponivel) {
    Serial.println(F("MPU-6050 conectado."));
    calibrarMPU();
  } else {
    Serial.println(F("MPU-6050 nao encontrado; inclinacao indisponivel."));
  }
  Serial.println(F("Sensores e alertas iniciados."));
}

void loop() {
  atualizarBotao();

  const unsigned long agora = millis();
  if (agora - ultimoCicloSensores >= INTERVALO_SENSORES_MS) {
    atualizarDistancias();
  }

  if (mpuDisponivel &&
      millis() - ultimaLeituraMPU >= INTERVALO_MPU_MS) {
    ultimaLeituraMPU = millis();
    lerAngulo(anguloAtual);
  }

  atualizarBuzzer();
  atualizarServo();

  if (millis() - ultimoStatus >= INTERVALO_STATUS_MS) {
    ultimoStatus = millis();
    imprimirStatus();
  }
}
