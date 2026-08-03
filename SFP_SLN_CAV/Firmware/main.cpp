/* * SIMULADOR DE FOTOPERIODO - FP_CS_AV - VERSÃO 1.2
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
#define PINO_DIMMER     3   // Pino PWM
#define PINO_VALVULA    9   // Relé da Válvula
#define PINO_TRIGGER    10  // Ultrassônico Gatilho (IO10/PB2)
#define PINO_ECHO       11  // Ultrassônico Retorno (IO11/PB3)

// --- PINOS DOS LEDs CFTV (Nível da Água) ---
#define LED_20          4
#define LED_40          5
#define LED_60          6
#define LED_80          7
#define LED_100         8

// --- CONFIGURAÇÕES DO SISTEMA ---
const int NIVEL_LUZ_ESCURO = 400; // Ajuste conforme o Trimpot
const int DISTANCIA_TANQUE_VAZIO = 100; // cm (Fundo do tanque)
const int DISTANCIA_TANQUE_CHEIO = 10;  // cm (Borda)
const int LIMITE_QUEDA_VAZAMENTO = 5;   // Se baixar 5cm em pouco tempo = VAZAMENTO

// --- OBJETOS ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 
RTC_DS3231 rtc; 

// --- VARIÁVEIS GLOBAIS ---
DateTime agora;
unsigned long ultimoCiclo = 0;
unsigned long ultimoTesteVazamento = 0;
int distanciaAnterior = 0;
int porcentagemAguaGlobal = 0; // Guarda o nível atual para os LEDs
bool vazamentoDetectado = false;
bool mostrarNivel = false; 
bool estadoLuz = false; // Flag para saber se a luz está ligada

// Ícone de Gota (apenas estética)
byte gota[8] = {0x04,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00};

void setup() {
  // Configura Pinos Sensores/Atuadores
  pinMode(PINO_DIMMER, OUTPUT);
  pinMode(PINO_VALVULA, OUTPUT);
  pinMode(PINO_TRIGGER, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  
  // Configura Pinos dos LEDs (CFTV)
  pinMode(LED_20, OUTPUT);
  pinMode(LED_40, OUTPUT);
  pinMode(LED_60, OUTPUT);
  pinMode(LED_80, OUTPUT);
  pinMode(LED_100, OUTPUT);
  
  // Botões com Pull-Up Interno
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
  digitalWrite(PINO_VALVULA, LOW); 
  analogWrite(PINO_DIMMER, 0);     
  
  // Leitura inicial estabilizada
  distanciaAnterior = lerUltrassonicoEstavel();
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();
}

void loop() {
  agora = rtc.now();
  
  // --- VERIFICAÇÃO DE BOTÕES ---
  if (digitalRead(BTN_UP) == LOW) {
    mostrarNivel = true; 
  } else {
    mostrarNivel = false; 
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
    atualizarBarraLEDsCFTV(); // Atualiza os LEDs
  }

  // --- VERIFICAÇÃO DE VAZAMENTO ---
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
    analogWrite(PINO_DIMMER, 255); 
    estadoLuz = true;
  } else {
    analogWrite(PINO_DIMMER, 0);   
    estadoLuz = false;
  }
}

// NOVA FUNÇÃO: Leitura estável com filtro de marola e timeout de segurança
int lerUltrassonicoEstavel() {
  long soma = 0;
  for(int i=0; i<3; i++) {
    digitalWrite(PINO_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(PINO_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIGGER, LOW);
    
    // TIMEOUT de 25000us (aprox 4 metros). Evita travar o microcontrolador!
    long duracao = pulseIn(PINO_ECHO, HIGH, 25000); 
    if(duracao == 0) duracao = 25000; // Se falhar, assume distância máxima
    
    soma += (duracao * 0.034 / 2);
    delay(10); // Pausa breve entre os pulsos
  }
  return soma / 3; // Retorna a média
}

void atualizarBarraLEDsCFTV() {
  int dist = lerUltrassonicoEstavel();
  // Calcula e salva na variável global para o LCD usar também
  porcentagemAguaGlobal = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagemAguaGlobal = constrain(porcentagemAguaGlobal, 0, 100);

  // Acende os LEDs em cascata conforme a porcentagem
  digitalWrite(LED_20, porcentagemAguaGlobal >= 20 ? HIGH : LOW);
  digitalWrite(LED_40, porcentagemAguaGlobal >= 40 ? HIGH : LOW);
  digitalWrite(LED_60, porcentagemAguaGlobal >= 60 ? HIGH : LOW);
  digitalWrite(LED_80, porcentagemAguaGlobal >= 80 ? HIGH : LOW);
  digitalWrite(LED_100, porcentagemAguaGlobal >= 95 ? HIGH : LOW); // 95 para garantir que acende perto do topo
}

void verificarVazamento() {
  if (vazamentoDetectado) return; 

  int distanciaAtual = lerUltrassonicoEstavel();
  int diferenca = distanciaAtual - distanciaAnterior;

  if (diferenca > LIMITE_QUEDA_VAZAMENTO) {
    vazamentoDetectado = true;
    digitalWrite(PINO_VALVULA, LOW); 
  } else {
    distanciaAnterior = distanciaAtual;
  }
}

// --- TELAS ---

void telaPrincipal() {
  lcd.setCursor(0, 0);
  if(agora.hour() < 10) lcd.print('0');
  lcd.print(agora.hour());
  lcd.print(':');
  if(agora.minute() < 10) lcd.print('0');
  lcd.print(agora.minute());
  
  lcd.setCursor(10, 0);
  lcd.print("Luz:");
  if(estadoLuz) lcd.print("ON ");
  else lcd.print("OFF");

  lcd.setCursor(0, 1);
  lcd.print("Status: OK      ");
}

void telaNivelAgua() {
  // Usa a variável global que já foi calculada pela função dos LEDs
  lcd.setCursor(0, 0);
  lcd.print("NIVEL DO TANQUE ");
  lcd.setCursor(0, 1);
  lcd.write(0); 
  lcd.print(" ");
  lcd.print(porcentagemAguaGlobal);
  lcd.print("%          "); 
}

void telaAlarme() {
  lcd.setCursor(0, 0);
  lcd.print("! ALARME VAZTO !");
  lcd.setCursor(0, 1);
  lcd.print("VALVULA FECHADA ");
  
  if ((millis() / 500) % 2 == 0) lcd.noBacklight();
  else lcd.backlight();
}
