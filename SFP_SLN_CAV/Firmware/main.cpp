/* * SIMULADOR DE FOTOPERIODO - FP_CS_AV - VERSÃO 1.3
 * Hardware: ATmega328P, DS3232, LCD I2C, LDR, HC-SR04, Relé, IRF3205.
 * Detecção de vazamento por taxa de variação de nível, sem sensor de fluxo.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Biblioteca para o LCD via I2C
#include <RTClib.h>            // Biblioteca do RTC DS3231
#include <avr/wdt.h>           // Biblioteca do Watchdog

// --- MAPEAMENTO DE PINOS ---
#define PINO_LDR        A0
#define BTN_MENU        A1
#define BTN_UP          A2  // UP / Consulta Nível
#define BTN_DOWN        A3  // DOWN
#define PINO_DIMMER     3   // Pino PWM
#define PINO_VALVULA    9   // Relé Válvula (Ativa em HIGH)
#define PINO_TRIGGER    10  // Ultrassônico Gatilho
#define PINO_ECHO       11  // Ultrassônico Retorno

// Endereço I2C do Módulo PCF8574 extra para LEDs CFTV
#define ENDERECO_PCF_LEDS 0x26 

// --- CONFIGURAÇÕES DO SISTEMA ---
const int NIVEL_LUZ_ESCURO = 400; 
const int DISTANCIA_TANQUE_VAZIO = 100; // cm
const int DISTANCIA_TANQUE_CHEIO = 10;  // cm
const int LIMITE_QUEDA_VAZAMENTO = 15;   // Aumentado para 15cm para teste em bancada ser mais fácil
const int NIVEL_PARA_ENCHER = 40;        // % para abrir a válvula
const int NIVEL_PARA_PARAR = 90;         // % para fechar a válvula

// --- OBJETOS ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 
RTC_DS3231 rtc; 

// --- VARIÁVEIS GLOBAIS ---
DateTime agora;
unsigned long ultimoCiclo = 0;
unsigned long ultimoTesteVazamento = 0;
unsigned long tempoBotaoMenu = 0;
int distanciaAnterior = 0;
int porcentagemAguaGlobal = 0; 
bool vazamentoDetectado = false;
bool mostrarNivelTemporario = false; 
bool estadoLuz = false;
bool estadoValvula = false;   // Flag para saber se a válvula deve estar aberta

// Máquina de Estados do Menu
// 0: Normal, 1: Ajuste Hora, 2: Ajuste Minuto
int modoMenu = 0; 
int horaAjuste, minutoAjuste;

// Ícone de Gota (Estética)
byte gota[8] = {0x04,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00};

void setup() {
  // Configura Pinos
  pinMode(PINO_DIMMER, OUTPUT);
  pinMode(PINO_VALVULA, OUTPUT);
  pinMode(PINO_TRIGGER, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  
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
  
  // SE O RTC PERDEU A HORA (Bateria acabou), AJUSTA PARA A HORA DA COMPILAÇÃO
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Estado Inicial Seguro
  digitalWrite(PINO_VALVULA, LOW); // Válvula Fechada
  analogWrite(PINO_DIMMER, 0);                             // Luz Apagada
  
  distanciaAnterior = lerUltrassonicoEstavel();
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();


  wdt_enable(WDTO_8S);                                     // Ativa o Watchdog para 8 segundos
}

void loop() {
  wdt_reset();                                             // Reseta o contador do Watchdog
  
  if (modoMenu == 0) {
    agora = rtc.now();                                     // Só lê o RTC se não estiver no menu de ajuste (evita que a hora mude enquanto é ajustada)
  }
  
  gerenciarBotoesEMenu();

  // --- ATUALIZAÇÃO DO DISPLAY (MÁQUINA DE ESTADOS) ---
  if (modoMenu > 0) {
    telaAjusteHora();                                       // Tela de Menu tem prioridade
  } else if (vazamentoDetectado) {
    telaAlarme();                                           // Alarme tem segunda prioridade
  } else if (mostrarNivelTemporario) {
    telaNivelAgua();                                        // Consulta temporária
  } else {
    telaPrincipal();                                        // Tela padrão
  }

  // --- LÓGICA DE CONTROLE (Roda a cada 1 segundo) ---
  if (millis() - ultimoCiclo > 1000) {
    ultimoCiclo = millis();
    
    if (!vazamentoDetectado && modoMenu == 0) {
      controlarLuz();
      controlarNivelAgua();                                 // Lógica para abrir/fechar válvula
    }
    
    atualizarBarraLEDsCFTV();                               // Tenta atualizar I2C extra (não trava se não houver módulo)
  }

  // --- VERIFICAÇÃO DE VAZAMENTO ---
  if (modoMenu == 0 && (millis() - ultimoTesteVazamento > 10000)) { 
    verificarVazamento();
    ultimoTesteVazamento = millis();
  }
}

// --- FUNÇÕES DE LÓGICA ---

void controlarLuz() {
  int leituraLDR = analogRead(PINO_LDR);
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

void controlarNivelAgua() {
                                                            // Lógica de encher o tanque (Histerese)
  if (porcentagemAguaGlobal < NIVEL_PARA_ENCHER && !estadoValvula) {
    estadoValvula = true;                                   // Precisa encher
  } else if (porcentagemAguaGlobal > NIVEL_PARA_PARAR && estadoValvula) {
    estadoValvula = false;                                  // Tanque cheio, para de encher
  }

                       
  if (estadoValvula) {                                      // Atua na Válvula física
    digitalWrite(PINO_VALVULA, HIGH);                       // ABRE Válvula (Sinal HIGH)
  } else {
    digitalWrite(PINO_VALVULA, LOW);                        // FECHA Válvula
  }
}

int lerUltrassonicoEstavel() {
  long soma = 0;
  for(int i=0; i<3; i++) {
    digitalWrite(PINO_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(PINO_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIGGER, LOW);
    
    long duracao = pulseIn(PINO_ECHO, HIGH, 30000);         // Timeout 30ms
    if(duracao == 0) duracao = 30000; 
    
    soma += (duracao * 0.034 / 2);
    delay(10); 
  }
  int dist = soma / 3;
                                                            // Trava para evitar leituras absurdas em bancada
  return constrain(dist, DISTANCIA_TANQUE_CHEIO - 5, DISTANCIA_TANQUE_VAZIO + 10);
}

void verificarVazamento() {
  if (vazamentoDetectado) return; 

  int distanciaAtual = lerUltrassonicoEstavel();
  int diferenca = distanciaAtual - distanciaAnterior;


  if (diferenca > LIMITE_QUEDA_VAZAMENTO) {                  // Se a água desceu (distância aumentou) mais que o limite
    vazamentoDetectado = true;
    estadoValvula = false;                                   // Cancela intenção de encher
    digitalWrite(PINO_VALVULA, LOW);                         // TRAVA Válvula FECHADA (Segurança)
  } else {
    distanciaAnterior = distanciaAtual;                      // Atualiza referência
  }
}

// --- FUNÇÕES DE INTERFACE (BOTÕES E MENU) ---

void gerenciarBotoesEMenu() {
                                                             // Leitura dos botões (Lógica Invertida: LOW = Pressionado)
  bool btnMenuPres = (digitalRead(BTN_MENU) == LOW);
  bool btnUpPres = (digitalRead(BTN_UP) == LOW);
  bool btnDownPres = (digitalRead(BTN_DOWN) == LOW);

                                                             // Lógica do Botão Menu (Entrar no menu e Resetar Alarme)
  if (btnMenuPres) {
    if (tempoBotaoMenu == 0) tempoBotaoMenu = millis();
    
                                                             // Resetar Alarme (Segurar MENU por 3 segundos)
    if (vazamentoDetectado && (millis() - tempoBotaoMenu > 3000)) {
      vazamentoDetectado = false;
      distanciaAnterior = lerUltrassonicoEstavel();          // Reinicia referência
      lcd.clear();
      lcd.print("ALARME RESETADO");
      delay(1000);
      lcd.clear();
      tempoBotaoMenu = 0;
      return;                                                // Sai da função para evitar entrar no menu logo em seguida
    }
  } else {
                                                             // Botão Solto. Se foi um clique rápido (< 2s) e não estava em alarme
    if (tempoBotaoMenu > 0 && (millis() - tempoBotaoMenu < 2000) && !vazamentoDetectado) {
      lcd.clear();                                           // Limpa a tela
      modoMenu++;
      if (modoMenu > 2) {                                    // Sai do Menu, salva a hora no RTC

        rtc.adjust(DateTime(agora.year(), agora.month(), agora.day(), horaAjuste, minutoAjuste, 0));
        modoMenu = 0;
        lcd.print("HORA SALVA!");
        delay(1000);
        lcd.clear();
      } else if (modoMenu == 1) {                            // Entra no Menu, carrega hora atual para ajuste

        horaAjuste = agora.hour();
        minutoAjuste = agora.minute();
      }
    }
    tempoBotaoMenu = 0;
  }

  // Gerenciar ações dentro do Menu ou na Tela Principal
  if (modoMenu == 1) {                                       // Ajuste Hora
    if (btnUpPres) { horaAjuste++; delay(150); }
    if (btnDownPres) { horaAjuste--; delay(150); }
    if (horaAjuste > 23) horaAjuste = 0;
    if (horaAjuste < 0) horaAjuste = 23;
  } 
  else if (modoMenu == 2) {                                  // Ajuste Minuto
    if (btnUpPres) { minutoAjuste++; delay(150); }
    if (btnDownPres) { minutoAjuste--; delay(150); }
    if (minutoAjuste > 59) minutoAjuste = 0;
    if (minutoAjuste < 0) minutoAjuste = 59;
  }
  else if (modoMenu == 0) {                                  // Fora do Menu

    if (btnUpPres) {                                         // Botão UP consulta nível temporariamente
      mostrarNivelTemporario = true;
    } else {

      if(mostrarNivelTemporario) lcd.clear();                // Se acabou de soltar o botão, limpa a tela
      mostrarNivelTemporario = false;
    }
  }
}

void atualizarBarraLEDsCFTV() {
  int dist = lerUltrassonicoEstavel();
  porcentagemAguaGlobal = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagemAguaGlobal = constrain(porcentagemAguaGlobal, 0, 100);

  // Lógica Invertida (0 = Aceso) para Sink do PCF8574
  byte estadoLeds = 0xFF;                                    // Tudo apagado (B11111111) inicialmente
  if (porcentagemAguaGlobal >= 20) estadoLeds &= ~B00000001; // Apenas 1 LED aceso, nível abaixo de 20%
  if (porcentagemAguaGlobal >= 40) estadoLeds &= ~B00000010; // 2 LEDs acesos, nível acima de 20% e abaixo de 40%
  if (porcentagemAguaGlobal >= 60) estadoLeds &= ~B00000100; // 3 LEDs acesos, nível acima de 40% e abaixo de 60%
  if (porcentagemAguaGlobal >= 80) estadoLeds &= ~B00001000; // 4 LEDs acesos, nível acima de 60% e abaixo de 80%
  if (porcentagemAguaGlobal >= 95) estadoLeds &= ~B00010000; // 5 LEDs acesos, nível acima de 80% e abaixo de 95%


  Wire.beginTransmission(ENDERECO_PCF_LEDS);                 // Envia via I2C (Não trava se o módulo não existir)
  Wire.write(estadoLeds);
  Wire.endTransmission();
}

// --- FUNÇÕES DE TELA (LCD COM FORMATAÇÃO FIXA) ---

void telaPrincipal() {
  lcd.setCursor(0, 0);
  if(agora.hour() < 10) lcd.print('0');
  lcd.print(agora.hour());
  lcd.print(':');
  if(agora.minute() < 10) lcd.print('0');
  lcd.print(agora.minute());
  

  lcd.print("      ");                            // Limpa o meio da linha com espaços

  lcd.setCursor(10, 0);
  lcd.print("Luz:");
  if(estadoLuz) lcd.print("ON ");
  else lcd.print("OFF");

  lcd.setCursor(0, 1);

  lcd.print("Status: OK      ");                  // Garante 16 caracteres para limpar linha antiga
}

void telaNivelAgua() {
  lcd.setCursor(0, 0);
  lcd.print("NIVEL DO TANQUE ");                  // 16 chars
  lcd.setCursor(0, 1);
  lcd.write(0);                                   // Gota
  lcd.print(" ");
  if(porcentagemAguaGlobal < 100) lcd.print(" "); // Alinhamento
  if(porcentagemAguaGlobal < 10) lcd.print(" ");  // Alinhamento
  lcd.print(porcentagemAguaGlobal);
  lcd.print("%          ");                       // Espaços para limpar final da linha antiga
}

void telaAlarme() {
  lcd.setCursor(0, 0);
  lcd.print("! ALARME VAZTO !");                 // 16 caracteres
  lcd.setCursor(0, 1);
  lcd.print("VALVULA FECHADA ");                 // 16 caracteres
  
  // Pisca Backlight
  if ((millis() / 500) % 2 == 0) lcd.noBacklight();
  else lcd.backlight();
}

void telaAjusteHora() {
  lcd.backlight();                               // Garante luz acesa no menu
  lcd.setCursor(0, 0);
  lcd.print("MENU: AJUSTAR   ");
  
  lcd.setCursor(0, 1);
  lcd.print("HORA: ");
  
  
  bool pisca = ((millis() / 300) % 2 == 0);      // Pisca o campo que está sendo ajustado

  
  if (modoMenu == 1 && pisca) lcd.print("  ");   // Campo Hora
  else {
    if(horaAjuste < 10) lcd.print('0');
    lcd.print(horaAjuste);
  }
  
  lcd.print(':');
  

  if (modoMenu == 2 && pisca) lcd.print("  ");  // Campo Minuto
  else {
    if(minutoAjuste < 10) lcd.print('0');
    lcd.print(minutoAjuste);
  }
  
  lcd.print("     ");                           // Limpa final da linha
}
