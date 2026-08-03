/* * SIMULADOR DE FOTOPERIODO - FP_CS_AV - VERSÃO 2.0
 * Hardware: ATmega328P, DS3232, LCD I2C, LDR, HC-SR04, Relé, IRF3205.
 * Detecção de vazamento por taxa de variação de nível, sem sensor de fluxo.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <avr/wdt.h> // Watchdog

// --- MAPEAMENTO DE PINOS ---
#define PINO_LDR        A0
#define BTN_MENU        A1  // MENU/AJUSTE
#define BTN_UP          A2  // UP / Consulta Nível
#define BTN_DOWN        A3  // DOWN
#define PINO_DIMMER     3   // Pino PWM
#define PINO_VALVULA    9   // Relé Válvula
#define PINO_TRIGGER    10  // Ultrassônico Gatilho
#define PINO_ECHO       11  // Ultrassônico Retorno

// Endereço I2C do Módulo PCF8574 extra para LEDs
#define ENDERECO_PCF_LEDS 0x26 

// --- CONFIGURAÇÕES DO SISTEMA ---
const int NIVEL_LUZ_ESCURO = 400; 
const int DISTANCIA_TANQUE_VAZIO = 100; // cm
const int DISTANCIA_TANQUE_CHEIO = 10;  // cm

const int LIMITE_QUEDA_VAZAMENTO = 10;  // Diminuído para 10cm para facilitar teste em bancada
const unsigned long tempoDebounce = 50; // Tempo em ms para aceitar o botão como pressionado (Debounce)

// --- TABELA GAMA (Memória Flash) ---
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

// --- OBJETOS ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 
RTC_DS3231 rtc; 

// --- VARIÁVEIS GLOBAIS ---
DateTime agora;
unsigned long ultimoCiclo = 0;
unsigned long ultimoTesteVazamento = 0;
int distanciaAnterior = 0;
int porcentagemAguaGlobal = 0; 
bool vazamentoDetectado = false;
bool estadoLuz = false;

// Estados estabilizados dos botões (pós-debounce)
bool menuPressionado = false;
bool upPressionado = false;
bool downPressionado = false;

// Flags de detecção de "borda de descida" (clique único)
bool menuFoiClicado = false;
bool upFoiClicado = false;
bool downFoiClicado = false;

// Máquina de Estados do Menu
// 0: Normal, 1: Ajuste Hora, 2: Ajuste Minuto
int modoMenu = 0; 
int horaAjuste, minutoAjuste;

// Ícone de Gota
byte gota[8] = {0x04,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00};


void setup() {
  // Configura Pinos Sensores/Atuadores
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
    while (1); // Trava se não achar RTC
  }
  
  // --- AJUSTE DE HORA AUTOMÁTICAMENTE - PRECISA DESCOMENTAR A LINHA ABAIXO
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // SE O RTC PERDEU A HORA (Bateria acabou)
  if (rtc.lostPower()) {
    lcd.print("Bateria RTC Ruim");
    delay(2000);
    lcd.clear();
  }

  // --- LÓGICA VÁLVULA ---
  // Inicia sempre DESLIGADA (Nível Baixo / LOW)
  digitalWrite(PINO_VALVULA, LOW); 
  vazamentoDetectado = false;

  analogWrite(PINO_DIMMER, 0);     // Luz Apagada
  
  // Leitura inicial estabilizada
  distanciaAnterior = lerUltrassonicoEstavel();
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();


  wdt_enable(WDTO_8S);  // Ativa o Watchdog para 8 segundos
}

void loop() {
  wdt_reset(); // Reinicia o Watchdog
  
  if (modoMenu == 0) {
    agora = rtc.now(); // Só lê o RTC se não estiver no menu de ajuste (pra não mudar a hora durante o ajuste)
  }
  
  // 1. Processa Debounce de todos os botões
  processarDebounceBotoes();
  
  // 2. Processa Lógica do Menu baseada nos cliques estáveis
  gerenciarMenuAjuste();

  // --- ATUALIZAÇÃO DO DISPLAY (MÁQUINA DE ESTADOS) ---
  if (modoMenu > 0) {
    telaAjusteHora(); // Tela de Menu tem prioridade
  } else if (vazamentoDetectado) {
    telaAlarme();    // Alarme tem segunda prioridade
  } else {
    // Tela Principal. Usa upPressionado (estável) para mostrar nível temporário
    telaPrincipal(upPressionado == LOW); 
  }

  // --- LÓGICA DE CONTROLE (Roda a cada 1 segundo) ---
  if (millis() - ultimoCiclo > 1000) {
    ultimoCiclo = millis();
    
    // Só controla luz e verifica vazamento fora do menu
    if (modoMenu == 0) {
      controlarLuz();
      
      // Garante que a válvula continua HIGH se não houver vazamento
      if (!vazamentoDetectado) {
        digitalWrite(PINO_VALVULA, HIGH); 
      }
    }
    
    atualizarBarraLEDsCFTV(); // Tenta atualizar I2C extra (não trava se não houver módulo)
  }

  // --- VERIFICAÇÃO DE VAZAMENTO ---
  if (modoMenu == 0 && (millis() - ultimoTesteVazamento > 10000)) { 
    verificarVazamento();
    ultimoTesteVazamento = millis();
  }
}

// --- FUNÇÃO DE CONTROLE DE LUZ COM RAMPA GAMA ---
void controlarLuz() {
  int leituraLDR = analogRead(PINO_LDR);
  bool estaEscuro = (leituraLDR < NIVEL_LUZ_ESCURO);

  // Parâmetros do Fotoperíodo
  const int HORA_LIGA = 17;
  const int HORA_DESLIGA = 21;
  const int MINUTOS_RAMPA = 15; // Duração do amanhecer/anoitecer

  // Converte a hora atual para "minutos desde a meia-noite" para facilitar o cálculo
  long minutosAtuais = agora.hour() * 60 + agora.minute();
  long inicioLuz = HORA_LIGA * 60;
  long fimLuz = HORA_DESLIGA * 60;

  int indiceLinearPWM = 0; // Índice de 0 a 255

  // Verifica se está dentro da janela de horário e se o LDR pede luz
  if (minutosAtuais >= inicioLuz && minutosAtuais < fimLuz && estaEscuro) {
    
    if (minutosAtuais < (inicioLuz + MINUTOS_RAMPA)) {
      // FASE 1: Rampa de Subida (Amanhecer)
      long segundosPassados = (minutosAtuais - inicioLuz) * 60 + agora.second();
      long segundosTotaisRampa = MINUTOS_RAMPA * 60;
      indiceLinearPWM = map(segundosPassados, 0, segundosTotaisRampa, 0, 255);
    } 
    else if (minutosAtuais >= (fimLuz - MINUTOS_RAMPA)) {
      // FASE 3: Rampa de Descida (Anoitecer)
      long segundosFaltando = (fimLuz * 60) - (minutosAtuais * 60 + agora.second());
      long segundosTotaisRampa = MINUTOS_RAMPA * 60;
      indiceLinearPWM = map(segundosFaltando, 0, segundosTotaisRampa, 0, 255);
    } 
    else {
      // FASE 2: Período central (Luz a 100%)
      indiceLinearPWM = 255;
    }
  } else {
    // Fora do horário ou dia claro
    indiceLinearPWM = 0;
  }

  // Trava de segurança para não extrapolar o índice do array
  indiceLinearPWM = constrain(indiceLinearPWM, 0, 255);

  // Lê o valor corrigido direto da memória Flash (PROGMEM)
  int valorPWMCorrigido = pgm_read_byte(&gamma8[indiceLinearPWM]);

  // Aplica o PWM no pino do MOSFET
  analogWrite(PINO_DIMMER, valorPWMCorrigido);
  
  // Atualiza o status para o display LCD
  estadoLuz = (valorPWMCorrigido > 0);
}

int lerUltrassonicoEstavel() {
  long soma = 0;
  for(int i=0; i<3; i++) {
    digitalWrite(PINO_TRIGGER, LOW);
    delayMicroseconds(2);
    digitalWrite(PINO_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINO_TRIGGER, LOW);
    
    long duracao = pulseIn(PINO_ECHO, HIGH, 30000); // Timeout 30ms
    if(duracao == 0) duracao = 30000; 
    
    soma += (duracao * 0.034 / 2);
    delay(10); 
  }
  return soma / 3;
}

void verificarVazamento() {
  if (vazamentoDetectado) return; 

  int distanciaAtual = lerUltrassonicoEstavel();
  int diferenca = distanciaAtual - distanciaAnterior;

  // Se a água desceu (distância aumentou) mais que o limite
  if (diferenca > LIMITE_QUEDA_VAZAMENTO) {
    vazamentoDetectado = true;

    digitalWrite(PINO_VALVULA, LOW);    // Desliga a bomba
  } else {
    distanciaAnterior = distanciaAtual; // Atualiza referência se variação normal
  }
}

// ---  ROTINA DE DEBOUNCE VIA SOFTWARE ---
void processarDebounceBotoes() {
  // Variáveis estáticas para guardar estado entre execuções do loop
  static unsigned long tempoUltimaMudancaMENU = 0;
  static unsigned long tempoUltimaMudancaUP = 0;
  static unsigned long tempoUltimaMudancaDOWN = 0;
  
  static bool ultimoEstadoCruMENU = HIGH;
  static bool ultimoEstadoCruUP = HIGH;
  static bool ultimoEstadoCruDOWN = HIGH;

  // Reseta as flags de clique único a cada loop
  menuFoiClicado = false;
  upFoiClicado = false;
  downFoiClicado = false;

  // --- DEBOUNCE BOTÃO MENU ---
  bool leituraAtualMENU = digitalRead(BTN_MENU); // Lê direto do pino
  if (leituraAtualMENU != ultimoEstadoCruMENU) {
    tempoUltimaMudancaMENU = millis(); // Reseta timer se houver ruído
  }
  if ((millis() - tempoUltimaMudancaMENU) > tempoDebounce) {
    // Sinal estabilizou. Verifica se mudou o estado final.
    if (leituraAtualMENU != menuPressionado) {
      menuPressionado = leituraAtualMENU; // LOW = Pressionado, HIGH = Solto
      // Detecta a "borda de descida" (momento exato que aperta)
      if (menuPressionado == LOW) menuFoiClicado = true;
    }
  }
  ultimoEstadoCruMENU = leituraAtualMENU;

  // --- DEBOUNCE BOTÃO UP ---
  bool leituraAtualUP = digitalRead(BTN_UP);
  if (leituraAtualUP != ultimoEstadoCruUP) {
    tempoUltimaMudancaUP = millis();
  }
  if ((millis() - tempoUltimaMudancaUP) > tempoDebounce) {
    if (leituraAtualUP != upPressionado) {
      upPressionado = leituraAtualUP;
      if (upPressionado == LOW) upFoiClicado = true;
    }
  }
  ultimoEstadoCruUP = leituraAtualUP;

  // --- DEBOUNCE BOTÃO DOWN ---
  bool leituraAtualDOWN = digitalRead(BTN_DOWN);
  if (leituraAtualDOWN != ultimoEstadoCruDOWN) {
    tempoUltimaMudancaDOWN = millis();
  }
  if ((millis() - tempoUltimaMudancaDOWN) > tempoDebounce) {
    if (leituraAtualDOWN != downPressionado) {
      downPressionado = leituraAtualDOWN;
      if (downPressionado == LOW) downFoiClicado = true;
    }
  }
  ultimoEstadoCruDOWN = leituraAtualDOWN;
}

// --- FUNÇÕES DE INTERFACE (MENU USANDO CLICKS ESTÁVEIS) ---

void gerenciarMenuAjuste() {
  
  // Lógica para Resetar Alarme (Segurar MENU por 3 segundos pós-debounce)
  static unsigned long tempoInicioSegurarMenu = 0;
  if (vazamentoDetectado) {
    if (menuPressionado == LOW) { // Segurando estabilizado
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      if ((millis() - tempoInicioSegurarMenu) > 3000) {
        // RESETAR ALARME E VÁLVULA
        vazamentoDetectado = false;
        digitalWrite(PINO_VALVULA, HIGH); // Volta a ligar a bomba
        distanciaAnterior = lerUltrassonicoEstavel(); 
        lcd.clear();
        lcd.print("ALARME RESETADO");
        delay(1000);
        lcd.clear();
        tempoInicioSegurarMenu = 0;
        return;
      }
    } else {
      tempoInicioSegurarMenu = 0;
    }
  }


  if (vazamentoDetectado) return;   // Se houver Alarme ativo, ignora o menu de ajuste de hora

  // Lógica de navegação do Menu (cliques únicos)
  if (menuFoiClicado) {
    lcd.clear(); // Limpa a tela
    modoMenu++;
    if (modoMenu > 2) {
      // Salva no RTC
      rtc.adjust(DateTime(agora.year(), agora.month(), agora.day(), horaAjuste, minutoAjuste, 0));
      modoMenu = 0;
      lcd.print("HORA SALVA!");
      delay(1000);
      lcd.clear();
    } else if (modoMenu == 1) {
      horaAjuste = agora.hour();
      minutoAjuste = agora.minute();
    }
  }

  // Gerenciar ações dentro do Menu usando cliques estabilizados
  if (modoMenu == 1) { // Ajuste da Hora
    if (upFoiClicado) horaAjuste++; 
    if (downFoiClicado) horaAjuste--; 
    if (horaAjuste > 23) horaAjuste = 0;
    if (horaAjuste < 0) horaAjuste = 23;
  } 
  else if (modoMenu == 2) { // Ajuste dos Minutos
    if (upFoiClicado) minutoAjuste++; 
    if (downFoiClicado) minutoAjuste--; 
    if (minutoAjuste > 59) minutoAjuste = 0;
    if (minutoAjuste < 0) minutoAjuste = 59;
  }
}

void atualizarBarraLEDsCFTV() {

  int dist = lerUltrassonicoEstavel();
  porcentagemAguaGlobal = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagemAguaGlobal = constrain(porcentagemAguaGlobal, 0, 100);

  Wire.beginTransmission(ENDERECO_PCF_LEDS);
  Wire.write(0xFF); // Temporário: apaga tudo se o módulo não existir
  Wire.endTransmission();
}

// --- FUNÇÕES DE TELA (LCD COM FORMATAÇÃO FIXA) ---

void telaPrincipal(bool mostrarNivel) {
  
  // Atualização da troca de tela (cm) sem usar lcd.clear()
  static bool ultimaVezMostrouNivel = false;
  if (!mostrarNivel && ultimaVezMostrouNivel) {
    lcd.setCursor(0, 1);
    lcd.print("                "); // Limpa linha 2
  }
  ultimaVezMostrouNivel = mostrarNivel;

  if (mostrarNivel) {
    // Tela Temporária de Nível (Segurando UP)
    lcd.setCursor(0, 0);
    lcd.print("NIVEL DO TANQUE "); // 16 caracteres
    lcd.setCursor(0, 1);
    lcd.write(0);                  // Gota
    lcd.print(" ");
    if(porcentagemAguaGlobal < 100) lcd.print(" "); // Alinhamento
    if(porcentagemAguaGlobal < 10) lcd.print(" ");  // Alinhamento
    lcd.print(porcentagemAguaGlobal);
    lcd.print("%          ");                       // Espaços para limpar final da linha antiga
  } 
  else {
    // Tela Padrão de Relógio
    lcd.setCursor(0, 0);
    if(agora.hour() < 10) lcd.print('0');    // Limpa o meio da linha com espaços
    lcd.print(agora.hour());
    lcd.print(':');
    if(agora.minute() < 10) lcd.print('0');
    lcd.print(agora.minute());
    lcd.print(':');
    if(agora.second() < 10) lcd.print('0');         // Adicionado segundos para ver rodar
    lcd.print(agora.second());
    
    lcd.print("    ");     // Limpa o meio da linha com espaços

    lcd.setCursor(10, 0);
    lcd.print("Luz:");
    if(estadoLuz) lcd.print("ON ");
    else lcd.print("OFF");

    lcd.setCursor(0, 1);
    lcd.print("Status: OK      "); 
  }
}

void telaAlarme() {
  lcd.setCursor(0, 0);
  lcd.print("! ALARME VAZTO !"); // 16 caracteres
  lcd.setCursor(0, 1);
  lcd.print("VALVULA FECHADA "); // 16 caracteres
  
  // Pisca Backlight usando millis
  if ((millis() / 500) % 2 == 0) lcd.noBacklight();
  else lcd.backlight();
}

void telaAjusteHora() {
  lcd.backlight(); // Garante luz acesa no menu
  lcd.setCursor(0, 0);
  lcd.print("MENU: AJUSTAR   ");
  
  lcd.setCursor(0, 1);
  lcd.print("HORA: ");
  
  bool pisca = ((millis() / 300) % 2 == 0);

  // Campo Hora
  if (modoMenu == 1 && pisca) lcd.print("  ");
  else {
    if(horaAjuste < 10) lcd.print('0');
    lcd.print(horaAjuste);
  }
  
  lcd.print(':');
  
  // Campo Minuto
  if (modoMenu == 2 && pisca) lcd.print("  ");
  else {
    if(minutoAjuste < 10) lcd.print('0');
    lcd.print(minutoAjuste);
  }
  
  lcd.print("     "); // Limpa final da linha
}
