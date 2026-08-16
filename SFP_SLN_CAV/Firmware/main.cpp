/* * SIMULADOR DE FOTOPERIODO - FP_CS_AV - VERSÃO 6.1
 * Hardware: ATmega328P, DS3231, LCD I2C, LDR, HC-SR04, YF-S201, Relé, IRF3205.
 * Detecção de vazamento através de sensor de fluxo.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <avr/wdt.h> // Watchdog
#include <EEPROM.h>  // Memória Não Volátil

// --- MAPEAMENTO DE PINOS ---
#define PINO_SENSOR_FLUXO 2   // Fio amarelo do YF-S201 (Obrigatório ser pino 2 ou 3 para interrupção)
#define PINO_LDR        A0    // Pino para o LDR
#define BTN_MENU        A1    // Botão menu
#define BTN_UP          A2    // Botão UP 
#define BTN_DOWN        A3    // Botão DOWN
#define PINO_DIMMER     3     // Pino do PWM
#define PINO_VALVULA    9     // Pino de ligar a válvula solenóide tippo NA
#define PINO_TRIGGER    10    // Pino do Sensor de Nível
#define PINO_ECHO       11    // Pino do sensor de nível

// Endereço I2C do Módulo PCF8574 extra para LEDs
#define ENDERECO_PCF_LEDS 0x26 

// --- CONFIGURAÇÕES DO SISTEMA ---
const int DISTANCIA_TANQUE_VAZIO = 100; // cm (Ajustar em campo)
const int DISTANCIA_TANQUE_CHEIO = 10;  // cm (Ajustar em campo)

// Limite de pulsos do Sensor de Fluxo a cada 10 segundos para considerar vazamento
// O YF-S201 gera aprox. 450 pulsos por litro. O valor abaixo deve ser ajustado na prática
const int LIMITE_PULSOS_VAZAMENTO = 50; 
const unsigned long tempoDebounce = 50; 

#define EEPROM_INIT_CODE 0x42 
#define EEPROM_ADDR_LDR  13   // Endereço para salvar o status do LDR

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
int porcentagemAguaGlobal = 0; 
int pwmAtualGlobal = 0;                   // Armazena o valor do Gama atual (0 a 255)
bool vazamentoDetectado = false;
bool estadoLuz = false;

// Contador de pulsos do sensor de fluxo (volatile porque roda na interrupção)
volatile unsigned int contadorPulsosFluxo = 0;

// Variáveis LDR Manual Override
bool sensorLuzAtivo = true; 
unsigned long tempoBotoesPressione = 0;
bool comboPressionado = false;

// Variáveis para as Telas "Pop-up" Temporárias (3 segundos)
byte telaTemporariaAtiva = 0; // 0=Nenhuma, 1=Água, 2=Gama
unsigned long tempoExibicaoTela = 0;

// Variáveis do Cronograma (Armazenadas em Minutos desde a Meia-Noite)
int t1_amanhecerIni = 240;   // 04:00 Padrão
int t2_amanhecerFim = 270;   // 04:30 Padrão
int t3_diaProporcional = 480;// 08:00 Padrão
int t4_tarde100 = 960;       // 16:00 Padrão
int t5_anoitecerIni = 1200;  // 20:00 Padrão
int t6_anoitecerFim = 1230;  // 20:30 Padrão 

// Estados estabilizados dos botões (pós-debounce)
bool menuPressionado = false;
bool upPressionado = false;
bool downPressionado = false;

// Flags de detecção de clique único
bool menuFoiClicado = false;
bool upFoiClicado = false;
bool downFoiClicado = false;

// Máquina de Estados do Menu
int modoMenu = 0; 
int horaAjuste, minutoAjuste;

// Ícone de Gota
byte gota[8] = {0x04,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00};

// --- FUNÇÃO DE INTERRUPÇÃO DO SENSOR DE FLUXO ---
void contarPulsos() {
  contadorPulsosFluxo++;
}

void setup() {
  // Configura Pinos Sensores/Atuadores
  pinMode(PINO_DIMMER, OUTPUT);
  pinMode(PINO_VALVULA, OUTPUT);
  pinMode(PINO_TRIGGER, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  
  // Configura o sensor de fluxo e atrela a interrupção
  pinMode(PINO_SENSOR_FLUXO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PINO_SENSOR_FLUXO), contarPulsos, FALLING);

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
    // COMENTAR A LINHA ABAIXO APÓS O PRIMEIRO UPLOAD
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // SE O RTC PERDEU A HORA (Bateria acabou)
  if (rtc.lostPower()) {
    lcd.print("Bateria RTC Ruim");
    delay(2000);
    lcd.clear();
  }

  // Inicialização EEPROM e Leitura dos Dados
  byte eepromStatus = EEPROM.read(0);
  if (eepromStatus != EEPROM_INIT_CODE) {

    // É a primeira vez. Grava os padrões na EEPROM
    EEPROM.put(1, t1_amanhecerIni);
    EEPROM.put(3, t2_amanhecerFim);
    EEPROM.put(5, t3_diaProporcional);
    EEPROM.put(7, t4_tarde100);
    EEPROM.put(9, t5_anoitecerIni);
    EEPROM.put(11, t6_anoitecerFim);
 
    // Configuração inicial do LDR salva como ATIVO (1)
    EEPROM.write(EEPROM_ADDR_LDR, 1); 

    EEPROM.write(0, EEPROM_INIT_CODE); // Salva a formatação
  } else {
    // Já formatado. Lê as configurações salvas do usuário
    EEPROM.get(1, t1_amanhecerIni);
    EEPROM.get(3, t2_amanhecerFim);
    EEPROM.get(5, t3_diaProporcional);
    EEPROM.get(7, t4_tarde100);
    EEPROM.get(9, t5_anoitecerIni);
    EEPROM.get(11, t6_anoitecerFim);
    
    // Recupera o status do LDR salvo antes de reiniciar
    sensorLuzAtivo = EEPROM.read(EEPROM_ADDR_LDR) == 1; 
  }

  // --- LÓGICA VÁLVULA ---
  // Inicia sempre DESLIGADA (Nível Baixo / LOW)
  digitalWrite(PINO_VALVULA, LOW); 
  vazamentoDetectado = false;

  analogWrite(PINO_DIMMER, 0);    // Luz Apagada 

  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();

  wdt_enable(WDTO_8S);   // Ativa o Watchdog para 8 segundos
}

void loop() {
  wdt_reset(); // Reinicia o Watchdog
  
  if (modoMenu == 0) agora = rtc.now(); 

  // --- OVERRIDE MANUAL: LIGAR/DESLIGAR SENSOR DE LUZ (UP + DOWN por 1s) ---
  bool btnUpCru = (digitalRead(BTN_UP) == LOW);
  bool btnDownCru = (digitalRead(BTN_DOWN) == LOW);

  if (btnUpCru && btnDownCru && modoMenu == 0 && !vazamentoDetectado) {
    if (!comboPressionado) {
      comboPressionado = true;
      tempoBotoesPressione = millis();
    } 
    else if (millis() - tempoBotoesPressione >= 1000) {
      sensorLuzAtivo = !sensorLuzAtivo; // Inverte o status do LDR
      
      // Salva a nova preferência na EEPROM imediatamente
      EEPROM.write(EEPROM_ADDR_LDR, sensorLuzAtivo ? 1 : 0);
      
      lcd.clear();
      lcd.backlight();
      if (sensorLuzAtivo) {
        lcd.print("SENSOR DE LUZ ON"); 
      } else {
        lcd.print("SENSOR DE LUZ OF"); 
      }
      
      delay(2000); 
      lcd.clear();
      
      // Trava de segurança: Espera que os botões sejam soltos
      while(digitalRead(BTN_UP) == LOW || digitalRead(BTN_DOWN) == LOW) {
        wdt_reset(); // Alimenta o Watchdog pra a placa não resetar
        delay(10);
      }
      comboPressionado = false; 
    }
  } else {
    comboPressionado = false; 
  }

   // 1. Processa Debounce de todos os botões
  processarDebounceBotoes();
  
  // 2. Processa Lógica do Menu baseada nos cliques estáveis
  gerenciarMenuAjuste();

  // --- MÁQUINA DE ESTADOS DO DISPLAY ---
  if (modoMenu > 0) {
    telaAjusteHora(); // Tela de Menu tem prioridade
  } else if (vazamentoDetectado) {
    telaAlarme();      // Alarme tem segunda prioridade
  } else {
    // Telas temporárias de 3 segundos
    if (telaTemporariaAtiva > 0) {
      if (millis() - tempoExibicaoTela > 3000) {
        telaTemporariaAtiva = 0; // Acabou o tempo
        lcd.clear(); // Limpa a tela para o relógio entrar limpo
      } else {
        if (telaTemporariaAtiva == 1) telaNivelAgua();
        else if (telaTemporariaAtiva == 2) telaNivelGama();
      }
    } else {
      telaPrincipal(); 
    }
  }

  // --- CICLO PRINCIPAL (1 Segundo) ---
  if (millis() - ultimoCiclo > 1000) {
    ultimoCiclo = millis();
    
    if (modoMenu == 0) {
      controlarLuz();
    }
    
    atualizarBarraLEDsCFTV(); // Lê o ultrassônico e calcula porcentagemAguaGlobal
  }

  // --- VERIFICAÇÃO DE VAZAMENTO (A cada 10s) ---
  if (modoMenu == 0 && (millis() - ultimoTesteVazamento > 10000)) { 
    verificarVazamento();
    ultimoTesteVazamento = millis();
  }
}

// --- FUNÇÃO DE VAZAMENTO USANDO YF-S201 ---
void verificarVazamento() {
  if (vazamentoDetectado) return; 

  // Desliga interrupções rapidamente apenas para ler e zerar a variável global com segurança
  noInterrupts();
  unsigned int pulsosAtuais = contadorPulsosFluxo;
  contadorPulsosFluxo = 0;
  interrupts();

  // Se o número de giros da hélice nos últimos 10 segundos passar do limite
  if (pulsosAtuais > LIMITE_PULSOS_VAZAMENTO) {
    vazamentoDetectado = true;
    digitalWrite(PINO_VALVULA, HIGH); // Aciona o relé para ligar a válvula e fechar a passagem da água
  }
}

// --- FOTOPERÍODO E GAMA ---
void controlarLuz() {
  long minutosAtuais = agora.hour() * 60 + agora.minute();
  long segundosDoDia = minutosAtuais * 60 + agora.second();

  int indiceLinearPWM = 0; // De 0 a 255

  // 1. 04:00 às 04:30 (Amanhecer)
  if (minutosAtuais >= t1_amanhecerIni && minutosAtuais < t2_amanhecerFim) {
    long segundosPassados = segundosDoDia - (t1_amanhecerIni * 60L);
    long segundosTotaisRampa = (t2_amanhecerFim - t1_amanhecerIni) * 60L; 
    indiceLinearPWM = map(segundosPassados, 0, segundosTotaisRampa, 0, 255);
  }
  // 2. 04:30 às 08:00 (Fixo 100%)
  else if (minutosAtuais >= t2_amanhecerFim && minutosAtuais < t3_diaProporcional) {
    indiceLinearPWM = 255;
  }
  // 3. 08:00 às 16:00 (LDR Ativo OU Fixo 0%)
  else if (minutosAtuais >= t3_diaProporcional && minutosAtuais < t4_tarde100) {
    if (sensorLuzAtivo) {
      static float pwmIntegrativo = 255; 
      static unsigned long tempoUltimaAcao = millis();

      const int ALVO_LDR = 400;   // Ajuste do alvo no trimpot
      const int JANELA = 80;      

      if (millis() - tempoUltimaAcao >= 100) { 
        int leituraLDR = analogRead(PINO_LDR);
        
        // Escuro = leituraLDR BAIXA / Claro = leituraLDR ALTA
        if (leituraLDR < (ALVO_LDR - JANELA)) {
          pwmIntegrativo += 0.5; // Escureceu, aumenta a luz
        } 
        else if (leituraLDR > (ALVO_LDR + JANELA)) {
          pwmIntegrativo -= 0.5; // Clareou, diminui a luz
        }

        if (pwmIntegrativo > 255) pwmIntegrativo = 255;
        if (pwmIntegrativo < 0) pwmIntegrativo = 0;
        
        tempoUltimaAcao = millis();
      }
      indiceLinearPWM = (int)pwmIntegrativo;
    } 
    else {
      // REGRA: Se o LDR foi desativado pelo usuario, a luz fica em 0 (Desligada)
      indiceLinearPWM = 0; 
    }
  }
  // 4. 16:00 às 20:00 (Fixo 100% independente do LDR)
  else if (minutosAtuais >= t4_tarde100 && minutosAtuais < t5_anoitecerIni) {
    indiceLinearPWM = 255;
  }
  // 5. 20:00 às 20:30 (Anoitecer)
  else if (minutosAtuais >= t5_anoitecerIni && minutosAtuais < t6_anoitecerFim) {
    long segundosPassados = segundosDoDia - (t5_anoitecerIni * 60L);
    long segundosTotaisRampa = (t6_anoitecerFim - t5_anoitecerIni) * 60L;
    indiceLinearPWM = map(segundosPassados, 0, segundosTotaisRampa, 255, 0); 
  }
  // 6. 20:30 às 04:00 (Madrugada - Tudo desligado)
  else {
    indiceLinearPWM = 0;
  }

  indiceLinearPWM = constrain(indiceLinearPWM, 0, 255);
  pwmAtualGlobal = pgm_read_byte(&gamma8[indiceLinearPWM]);
  analogWrite(PINO_DIMMER, pwmAtualGlobal);
  estadoLuz = (pwmAtualGlobal > 0);
}

// --- HARDWARE E SENSORES ---
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

// --- BOTÕES E MENU INTELIGENTE ---
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
  bool leituraAtualMENU = digitalRead(BTN_MENU);  // Lê direto do pino
  if (leituraAtualMENU != ultimoEstadoCruMENU) {
    tempoUltimaMudancaMENU = millis();            // Reseta timer se houver ruído
  }
  if ((millis() - tempoUltimaMudancaMENU) > tempoDebounce) {
    // Sinal estabilizou. Verifica se mudou o estado final.
    if (leituraAtualMENU != menuPressionado) {
      menuPressionado = leituraAtualMENU;         // LOW = Pressionado, HIGH = Solto
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


void ajustarVariavelTempo(int &variavelTempo, bool aumenta) {
  if (aumenta) variavelTempo += 5;
  else variavelTempo -= 5;
  if (variavelTempo > 1439) variavelTempo = 0;
  if (variavelTempo < 0) variavelTempo = 1435;
}

// --- FUNÇÕES DE INTERFACE (MENU USANDO CLICKS ESTÁVEIS) ---
void gerenciarMenuAjuste() {

  // Lógica para Resetar Alarme (Segurar MENU por 3 segundos pós-debounce)
  static unsigned long tempoInicioSegurarMenu = 0;
  static bool salvamentoExecutado = false;
  static bool ignorarSoltura = false; // Flag de escudo

  if (vazamentoDetectado) {
    if (menuPressionado == LOW) { 
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      if ((millis() - tempoInicioSegurarMenu) > 3000) {
        vazamentoDetectado = false;

        digitalWrite(PINO_VALVULA, LOW); // Volta a desligar a válvula
        
        // Zera os pulsos antes de rearmar
        noInterrupts();
        contadorPulsosFluxo = 0;
        interrupts();
        
        lcd.clear();
        lcd.backlight();
        lcd.print("ALARME RESETADO");
        delay(1000);
        lcd.clear();
        tempoInicioSegurarMenu = 0;
      }
    } else {
      tempoInicioSegurarMenu = 0;
    }
    return; // Impede de entrar no menu se estiver em alarme
  }

  // 2. TELA PRINCIPAL
  if (modoMenu == 0) {
    if (menuPressionado == LOW) {
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      if ((millis() - tempoInicioSegurarMenu) > 1000) {
        lcd.clear();
        modoMenu = 1;
        horaAjuste = agora.hour();
        minutoAjuste = agora.minute();
        tempoInicioSegurarMenu = 0;
        ignorarSoltura = true; // Avisa o sistema para ignorar quando soltar o dedo
      }
    } else {
      tempoInicioSegurarMenu = 0;
    }

    // Só exibe as telas rápidas se não estiver segurando o combo LDR (UP+DOWN)
    if (upFoiClicado && !comboPressionado) {
      telaTemporariaAtiva = 1; // Tela Água
      tempoExibicaoTela = millis();
      lcd.clear();
    }
    if (downFoiClicado && !comboPressionado) {
      telaTemporariaAtiva = 2; // Tela Gama
      tempoExibicaoTela = millis();
      lcd.clear();
    }
  } 
    // 3. DENTRO DO MENU
  else {
    // Avançar Telas (1 clique simples)
    if (menuPressionado == LOW) {
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      
      // Salvamento rápido agora zera o cronômetro antes de sair
      if ((millis() - tempoInicioSegurarMenu) > 1000 && !salvamentoExecutado && !ignorarSoltura) {
        salvamentoExecutado = true; 
        
        rtc.adjust(DateTime(agora.year(), agora.month(), agora.day(), horaAjuste, minutoAjuste, 0));
        EEPROM.put(1, t1_amanhecerIni);
        EEPROM.put(3, t2_amanhecerFim);
        EEPROM.put(5, t3_diaProporcional);
        EEPROM.put(7, t4_tarde100);
        EEPROM.put(9, t5_anoitecerIni);
        EEPROM.put(11, t6_anoitecerFim);
        
        lcd.clear();
        lcd.print("ALTERACAO  SALVA");
        lcd.setCursor(0, 1);
        lcd.print("  COM SUCESSO!  :)");
        delay(3000);
        
        modoMenu = 0;
        tempoInicioSegurarMenu = 0; 
        menuFoiClicado = false;     
        lcd.clear();
        return;                     
      }
    } 
    else { 
      // --- BOTÃO FOI SOLTO ---
      if (ignorarSoltura) {
        ignorarSoltura = false; 
      } 
      else if (tempoInicioSegurarMenu > 0) {
        unsigned long duracao = millis() - tempoInicioSegurarMenu;
        if (duracao < 1000 && !salvamentoExecutado) {
          lcd.clear(); 
          modoMenu++;
          if (modoMenu > 8) {
            rtc.adjust(DateTime(agora.year(), agora.month(), agora.day(), horaAjuste, minutoAjuste, 0));
            EEPROM.put(1, t1_amanhecerIni);
            EEPROM.put(3, t2_amanhecerFim);
            EEPROM.put(5, t3_diaProporcional);
            EEPROM.put(7, t4_tarde100);
            EEPROM.put(9, t5_anoitecerIni);
            EEPROM.put(11, t6_anoitecerFim);
            modoMenu = 0;
            lcd.print("SUAS ALTERACOES ");
            lcd.setCursor(0, 1);
            lcd.print("FORAM SALVAS! :)");
            delay(3000);
            lcd.clear();
          }
        }
      }
      tempoInicioSegurarMenu = 0;
      salvamentoExecutado = false;
    }

    if (modoMenu == 1) { 
      if (upFoiClicado) { horaAjuste++; if(horaAjuste > 23) horaAjuste = 0; }
      if (downFoiClicado) { horaAjuste--; if(horaAjuste < 0) horaAjuste = 23; }
    } else if (modoMenu == 2) { 
      if (upFoiClicado) { minutoAjuste++; if(minutoAjuste > 59) minutoAjuste = 0; }
      if (downFoiClicado) { minutoAjuste--; if(minutoAjuste < 0) minutoAjuste = 59; }
    } else if (modoMenu == 3) { if (upFoiClicado) ajustarVariavelTempo(t1_amanhecerIni, true); if (downFoiClicado) ajustarVariavelTempo(t1_amanhecerIni, false);
    } else if (modoMenu == 4) { if (upFoiClicado) ajustarVariavelTempo(t2_amanhecerFim, true); if (downFoiClicado) ajustarVariavelTempo(t2_amanhecerFim, false);
    } else if (modoMenu == 5) { if (upFoiClicado) ajustarVariavelTempo(t3_diaProporcional, true); if (downFoiClicado) ajustarVariavelTempo(t3_diaProporcional, false);
    } else if (modoMenu == 6) { if (upFoiClicado) ajustarVariavelTempo(t4_tarde100, true); if (downFoiClicado) ajustarVariavelTempo(t4_tarde100, false);
    } else if (modoMenu == 7) { if (upFoiClicado) ajustarVariavelTempo(t5_anoitecerIni, true); if (downFoiClicado) ajustarVariavelTempo(t5_anoitecerIni, false);
    } else if (modoMenu == 8) { if (upFoiClicado) ajustarVariavelTempo(t6_anoitecerFim, true); if (downFoiClicado) ajustarVariavelTempo(t6_anoitecerFim, false);
    }
  }
}

// --- FUNÇÕES DE EXIBIÇÃO NO LCD ---
// Função utilitária para desenhar HH:MM a partir de minutos totais
void printHoraFormatada(int minutosTotais) {
  int h = minutosTotais / 60;
  int m = minutosTotais % 60;
  if(h < 10) lcd.print('0');
  lcd.print(h);
  lcd.print(':');
  if(m < 10) lcd.print('0');
  lcd.print(m);
}

void telaAjusteHora() {
  lcd.backlight(); 
  lcd.setCursor(0, 0);
  
  if (modoMenu == 1 || modoMenu == 2) lcd.print("AJUSTAR RELOGIO ");
  else if (modoMenu == 3) lcd.print("1.AMANHECER INI ");
  else if (modoMenu == 4) lcd.print("2.AMANHECER FIM ");
  else if (modoMenu == 5) lcd.print("3.LIGAR SENSOR   ");
  else if (modoMenu == 6) lcd.print("4.LUZ 100% TARDE");
  else if (modoMenu == 7) lcd.print("5.ANOITECER INI ");
  else if (modoMenu == 8) lcd.print("6.ANOITECER FIM ");
  
  lcd.setCursor(0, 1);
  lcd.print("HORA: ");
  bool pisca = ((millis() / 300) % 2 == 0);

  if (modoMenu == 1 || modoMenu == 2) {
    if (modoMenu == 1 && pisca) lcd.print("  ");
    else { if(horaAjuste < 10) lcd.print('0'); lcd.print(horaAjuste); }
    lcd.print(':');
    if (modoMenu == 2 && pisca) lcd.print("  ");
    else { if(minutoAjuste < 10) lcd.print('0'); lcd.print(minutoAjuste); }
  } 
  else {
    // Exibe os horários do cronograma piscando
    if (pisca) lcd.print("     ");
    else {
      if (modoMenu == 3) printHoraFormatada(t1_amanhecerIni);
      else if (modoMenu == 4) printHoraFormatada(t2_amanhecerFim);
      else if (modoMenu == 5) printHoraFormatada(t3_diaProporcional);
      else if (modoMenu == 6) printHoraFormatada(t4_tarde100);
      else if (modoMenu == 7) printHoraFormatada(t5_anoitecerIni);
      else if (modoMenu == 8) printHoraFormatada(t6_anoitecerFim);
    }
  }
  lcd.print("     "); 
}

void atualizarBarraLEDsCFTV() {
  int dist = lerUltrassonicoEstavel();
  porcentagemAguaGlobal = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagemAguaGlobal = constrain(porcentagemAguaGlobal, 0, 100);

  Wire.beginTransmission(ENDERECO_PCF_LEDS);
  Wire.write(0xFF); 
  Wire.endTransmission();
}

void telaPrincipal() {
  lcd.setCursor(0, 0);
  if(agora.hour() < 10) lcd.print('0');
  lcd.print(agora.hour());
  lcd.print(':');
  if(agora.minute() < 10) lcd.print('0');
  lcd.print(agora.minute());
  lcd.print(':');
  if(agora.second() < 10) lcd.print('0'); 
  lcd.print(agora.second());
  
  lcd.print("  "); 
  lcd.setCursor(10, 0);
  lcd.print("Luz:");
  if(estadoLuz) lcd.print("ON ");
  else lcd.print("OFF");

// Painel dinâmico da segunda linha
  lcd.setCursor(0, 1);
  long minutosAtuais = agora.hour() * 60 + agora.minute();

  if (minutosAtuais >= t1_amanhecerIni && minutosAtuais < t2_amanhecerFim) {
    lcd.print("    Amanhecer   ");
  } 
  else if (minutosAtuais >= t2_amanhecerFim && minutosAtuais < t3_diaProporcional) {
    lcd.print(" Inicio da Manha"); 
  } 
  else if (minutosAtuais >= t3_diaProporcional && minutosAtuais < t4_tarde100) {
    if (sensorLuzAtivo) lcd.print(" Dia: Sensor ON ");
    else lcd.print(" Dia: Sensor OF ");
  } 
  else if (minutosAtuais >= t4_tarde100 && minutosAtuais < t5_anoitecerIni) {
    lcd.print("  Fim de Tarde  ");
  } 
  else if (minutosAtuais >= t5_anoitecerIni && minutosAtuais < t6_anoitecerFim) {
    lcd.print("    Anoitecer   ");
  } 
  else {
    lcd.print("     Noite      ");
  }
}

void telaNivelAgua() {
  lcd.setCursor(0, 0);
  lcd.print("NIVEL DA CAIXA: "); 
  
  lcd.setCursor(0, 1);
  lcd.write(0); 
  lcd.print(" ");
  if(porcentagemAguaGlobal < 100) lcd.print(" "); 
  if(porcentagemAguaGlobal < 10) lcd.print(" ");  
  lcd.print(porcentagemAguaGlobal);
  lcd.print("%          "); 
}

void telaNivelGama() {
  lcd.setCursor(0, 0);
  lcd.print(" NIVEL DE LUZ:  "); 
  lcd.setCursor(0, 1);
  lcd.print(" PWM:");
  
  int percLuz = map(pwmAtualGlobal, 0, 255, 0, 100);
  if (percLuz < 100) lcd.print(" "); 
  if (percLuz < 10) lcd.print(" ");  
  lcd.print(percLuz);
  lcd.print("% (");
  lcd.print(pwmAtualGlobal);
  lcd.print(")  "); 
}

void telaAlarme() {
  lcd.setCursor(0, 0);
  lcd.print("!  VAZAMENTO  ! "); 
  lcd.setCursor(0, 1);
  lcd.print("VALVULA FECHADA "); 
  if ((millis() / 500) % 2 == 0) lcd.noBacklight();
  else lcd.backlight();
}
