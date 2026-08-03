// SIMULADOR DE FOTOPERIODO - VERSÃO 1.0
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h> // Precisa instalar a biblioteca "RTClib by Adafruit"

RTC_DS3231 rtc;

// DEFINIÇÕES DE PINOS
const int pinoLED = 3; // Pino PWM

// TABELA DE CORREÇÃO GAMA (256 valores)
const uint8_t gamma8[] PROGMEM = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
};

void setup() {
  pinMode(pinoLED, OUTPUT);
  Serial.begin(9600);
  
  if (!rtc.begin()) {
    Serial.println("Erro: RTC nao encontrado!");
    while (1);
  }

  // Descomentar a linha abaixo APENAS UMA VEZ para acertar a hora do módulo. Depois, comentar novamente e fazer o upload de novo para não resetar a hora toda vez que ligar.
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  DateTime agora = rtc.now();
  
  // Converte a hora atual em "minutos totais desde a meia noite" para facilitar a conta
  // Ex: 04:00 = 240 minutos.
  int minutosDoDia = (agora.hour() * 60) + agora.minute();
  
  int nivelPWM = 0; // Padrão é apagado

  // --- LÓGICA DOS HORÁRIOS ---

  // 1. NASCER DO SOL (04:00 até 04:30) - Dura 30 min
  if (minutosDoDia >= 240 && minutosDoDia < 270) {
    // Mapeia o tempo atual (240 a 270) para uma escala de 0 a 255
    int progresso = map(minutosDoDia, 240, 269, 0, 255);
    nivelPWM = pgm_read_byte(&gamma8[progresso]); 
  }
  
  // 2. MANHÃ (04:30 até 08:00) - Luz Total
  else if (minutosDoDia >= 270 && minutosDoDia < 480) {
    nivelPWM = 255;
  }
  
  // 3. DIA (08:00 até 16:00) - Luz Desligada (Sol natural)
  else if (minutosDoDia >= 480 && minutosDoDia < 960) {
    nivelPWM = 0;
  }
  
  // 4. TARDE (16:00 até 19:30) - Luz Total (Preparo para anoitecer)
  else if (minutosDoDia >= 960 && minutosDoDia < 1170) {
    nivelPWM = 255;
  }
  
  // 5. PÔR DO SOL (19:30 até 20:00) - Dura 30 min
  else if (minutosDoDia >= 1170 && minutosDoDia < 1200) {
    // Mapeia o tempo atual (1170 a 1200) para escala INVERSA (255 a 0)
    int progresso = map(minutosDoDia, 1170, 1199, 255, 0);
    nivelPWM = pgm_read_byte(&gamma8[progresso]);
  }
  
  // 6. NOITE (20:00 até 04:00) - Apagado
  else {
    nivelPWM = 0;
  }

  // Envia o sinal para o MOSFET
  analogWrite(pinoLED, nivelPWM);
  
  // Debug para acompanhar no Monitor Serial (opcional)
  Serial.print(agora.hour());
  Serial.print(":");
  Serial.print(agora.minute());
  Serial.print(" | MinutosTotais: ");
  Serial.print(minutosDoDia);
  Serial.print(" | PWM: ");
  Serial.println(nivelPWM);

  delay(1000); // Atualiza a cada 1 segundo
}
