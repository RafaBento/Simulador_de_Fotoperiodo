/* * SIMULADOR DE FOTOPERIODO - FP_CS_AV - VERSÃO 1.1
 * Hardware: ATmega328P, DS3232, LCD I2C, LDR, HC-SR04, Relé, IRF3205.
 * Detecção de vazamento por taxa de variação de nível, sem sensor de fluxo.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// --- MAPEAMENTO DE PINOS ---
#define PINO_LDR        A0
#define BTN_MENU        A1
#define BTN_UP          A2  // Botão UP / Consulta Nível
#define BTN_DOWN        A3  // Botão DOWN
#define PINO_DIMMER     3   // PWM para MOSFET da Luz
#define PINO_VALVULA    9   // Relé da Válvula
#define PINO_TRIGGER    10  // Ultrassônico Gatilho (IO10/PB2)
#define PINO_ECHO       11  // Ultrassônico Retorno (IO11/PB3)

// --- CONFIGURAÇÕES DO SISTEMA ---
const int NIVEL_LUZ_ESCURO = 400; // Ajuste conforme o Trimpot
const int DISTANCIA_TANQUE_VAZIO = 100; // cm (Fundo do tanque)
const int DISTANCIA_TANQUE_CHEIO = 10;  // cm (Borda)
const int LIMITE_QUEDA_VAZAMENTO = 5;   // Se baixar 5cm em pouco tempo = VAZAMENTO

// --- OBJETOS ---
LiquidCrystal_I2C lcd(0x27, 16, 2); // Endereço 0x27, 16 Colunas e 2 Linhas
RTC_DS3231 rtc; // Foi preciso usar RTC_DS3231 porque a biblioteca nao reconeceu o DS3232, o resto do código segue inalterado

// --- VARIÁVEIS GLOBAIS ---
DateTime agora;
unsigned long ultimoCiclo = 0;
unsigned long ultimoTesteVazamento = 0;
int distanciaAnterior = 0;
bool vazamentoDetectado = false;
bool mostrarNivel = false; // Flag para mudar a tela temporariamente

// Ícone de Gota (Opcional, só estética)
byte gota[8] = {0x04,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00};

void setup() {
  // Configura Pinos
  pinMode(PINO_DIMMER, OUTPUT);
  pinMode(PINO_VALVULA, OUTPUT);
  pinMode(PINO_TRIGGER, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  
  // Botões com Pull-Up Interno (Lógica Invertida)
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  // Inicializa Periféricos
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, gota);

  if (!rtc.begin()) {
    lcd.print("Erro no RTC!");
    while (1);
  }

  // Estado Inicial
  digitalWrite(PINO_VALVULA, LOW); // Válvula Fechada por segurança
  analogWrite(PINO_DIMMER, 0);     // Luz Apagada
  
  // Leitura inicial para referência do vazamento
  distanciaAnterior = lerUltrassonico();
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(2000);
  lcd.clear();
}

void loop() {
  agora = rtc.now();
  
  // --- VERIFICAÇÃO DE BOTÕES ---
  if (digitalRead(BTN_UP) == LOW) {
    mostrarNivel = true; // Segurou UP: Mostra Nível
  } else {
    mostrarNivel = false; // Soltou UP: Volta pro Relógio
  }

  // --- ATUALIZAÇÃO DO DISPLAY ---
  if (mostrarNivel) {
    telaNivelAgua();
  } else if (vazamentoDetectado) {
    telaAlarme();
  } else {
    telaPrincipal();
  }

  // --- LÓGICA DE CONTROLE (Roda a cada 1 segundo) ---
  if (millis() - ultimoCiclo > 1000) {
    ultimoCiclo = millis();
    controlarLuz();
  }

  // --- VERIFICAÇÃO DE VAZAMENTO (Roda a cada N minutos - Ajustável) ---
  // Aqui coloquei 10 seg para testar. Ajuste final deve ser feito em campo
  if (millis() - ultimoTesteVazamento > 10000) { 
    verificarVazamento();
    ultimoTesteVazamento = millis();
  }
}

// --- FUNÇÕES AUXILIARES ---

void controlarLuz() {
  int leituraLDR = analogRead(PINO_LDR);
  
  // Exemplo: Luzes ligadas entre 17h e 21h SE estiver escuro
  bool horarioLuz = (agora.hour() >= 17 && agora.hour() < 21);
  bool estaEscuro = (leituraLDR < NIVEL_LUZ_ESCURO);

  if (horarioLuz && estaEscuro) {
    // Acende Suave (Opcional) ou Direto
    analogWrite(PINO_DIMMER, 255); // 100% (Basta usar um valor menor para dimerizar)
  } else {
    analogWrite(PINO_DIMMER, 0);   // Apaga
  }
}

int lerUltrassonico() {
  // Gera pulso de 10us
  digitalWrite(PINO_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PINO_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_TRIGGER, LOW);
  
  // Lê o tempo de retorno
  long duracao = pulseIn(PINO_ECHO, HIGH);
  int distancia = duracao * 0.034 / 2; // Converte para cm
  return distancia;
}

void verificarVazamento() {
  if (vazamentoDetectado) return; // Se já travou, não faz nada

  int distanciaAtual = lerUltrassonico();
  
  // Lógica: Se a distância AUMENTOU muito (nível da água desceu) e o tanque não estava enchendo
  int diferenca = distanciaAtual - distanciaAnterior;

  if (diferenca > LIMITE_QUEDA_VAZAMENTO) {
    // Nível caiu rápido demais!
    vazamentoDetectado = true;
    digitalWrite(PINO_VALVULA, LOW); // Trava Válvula FECHADA
  } else if (diferenca < 0) {
    // Nível subiu (está enchendo), atualiza referência
    distanciaAnterior = distanciaAtual; 
  } else {
    // Variação normal, atualiza referência lentamente
    distanciaAnterior = distanciaAtual;
  }
}

// --- TELAS ---

void telaPrincipal() {
  lcd.setCursor(0, 0);
  // Formata Hora: 12:05:00
  if(agora.hour() < 10) lcd.print('0');
  lcd.print(agora.hour());
  lcd.print(':');
  if(agora.minute() < 10) lcd.print('0');
  lcd.print(agora.minute());
  
  lcd.setCursor(10, 0);
  lcd.print("Luz:");
  // Mostra se a luz está ON ou OFF
  if(digitalRead(PINO_DIMMER)) lcd.print("ON ");
  else lcd.print("OFF");

  lcd.setCursor(0, 1);
  lcd.print("Status: OK      ");
}

void telaNivelAgua() {
  int dist = lerUltrassonico();
  // Converte cm para %. Supondo: 100cm = 0%, 10cm = 100%
  int porcentagem = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagem = constrain(porcentagem, 0, 100); // Trava entre 0 e 100

  lcd.setCursor(0, 0);
  lcd.print("NIVEL DO TANQUE ");
  lcd.setCursor(0, 1);
  lcd.write(0); // Ícone de gota
  lcd.print(" ");
  lcd.print(porcentagem);
  lcd.print("%  (");
  lcd.print(dist);
  lcd.print("cm)   ");
}

void telaAlarme() {
  lcd.setCursor(0, 0);
  lcd.print("! ALARME VAZTO !");
  lcd.setCursor(0, 1);
  lcd.print("VALVULA FECHADA ");
  
  // Pisca o Backlight para chamar atenção (Opcional)
  if ((millis() / 500) % 2 == 0) lcd.noBacklight();
  else lcd.backlight();
}
