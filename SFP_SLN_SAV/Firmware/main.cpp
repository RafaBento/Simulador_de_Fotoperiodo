/* * SIMULADOR DE FOTOPERIODO - SFP_SLN_SAV - VERSÃO 8.0
 * Hardware: ATmega328P, DS3231, LCD I2C, LDR, HC-SR04, Relé, IRF3205.
 * Detecção de nível com ultrassonico e acionamento da bomba que enche o reservatório via relé.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <avr/wdt.h> // Watchdog
#include <EEPROM.h>  // Memória Não Volátil

// --- MAPEAMENTO DE PINOS ---
#define PINO_LDR        A0    // Pino para o LDR
#define BTN_MENU        A1    // Botão menu
#define BTN_UP          A2    // Botão UP 
#define BTN_DOWN        A3    // Botão DOWN
#define PINO_DIMMER     3     // Pino do PWM
#define PINO_VALVULA    9     // Pino de ligar a bomba
#define PINO_TRIGGER    10    // Pino do Sensor de Nível
#define PINO_ECHO       11    // Pino do sensor de nível

// Endereço I2C do Módulo PCF8574 extra para LEDs
#define ENDERECO_PCF_LEDS 0x26 

// --- CONFIGURAÇÕES DO SISTEMA ---
const int DISTANCIA_TANQUE_VAZIO = 100; // cm (Ajustar em campo)
const int DISTANCIA_TANQUE_CHEIO = 10;  // cm (Ajustar em campo)
const unsigned long tempoDebounce = 50; 

#define EEPROM_INIT_CODE 0x42 
#define EEPROM_ADDR_LDR  13 // Endereço para salvar o status do LDR

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
unsigned long ultimoCicloLuz = 0;         // Cronômetro para a Luz (1s)
unsigned long ultimoCicloUltrassom = 0;   // Cronômetro para o Ultrassom/Bomba (5s)

int porcentagemAguaGlobal = 0; 
int pwmAtualGlobal = 0; 
bool estadoLuz = false;
bool valvulaLigada = false;

// --- VARIÁVEIS DE SEGURANÇA DA BOMBA ---
unsigned long tempoBombaLigada = 0;
const unsigned long TEMPO_MAXIMO_BOMBA = 600000; // 10 minutos em milissegundos (ajustar em campo)
bool erroBombaTravada = false; // FLAG DE ERRO


// Variáveis: LDR Manual Override
bool sensorLuzAtivo = true; 
unsigned long tempoBotoesPressione = 0;
bool comboPressionado = false;

// Variáveis para as Telas "Pop-up" Temporárias
byte telaTemporariaAtiva = 0; 
unsigned long tempoExibicaoTela = 0;

// Variáveis do Cronograma (Armazenadas em Minutos)
int t1_amanhecerIni = 240;   
int t2_amanhecerFim = 270;   
int t3_diaProporcional = 480;
int t4_tarde100 = 960;       
int t5_anoitecerIni = 1200;  
int t6_anoitecerFim = 1230;  

// Botoes e Menu
bool menuPressionado = false;
bool upPressionado = false;
bool downPressionado = false;
bool menuFoiClicado = false;
bool upFoiClicado = false;
bool downFoiClicado = false;
int modoMenu = 0; 
int horaAjuste, minutoAjuste;

byte gota[8] = {0x04,0x0E,0x1F,0x1F,0x1F,0x0E,0x00,0x00}; //apenas para enfeite no display

void setup() {
  pinMode(PINO_DIMMER, OUTPUT);
  pinMode(PINO_VALVULA, OUTPUT);
  pinMode(PINO_TRIGGER, OUTPUT);
  pinMode(PINO_ECHO, INPUT);
  
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, gota);

  if (!rtc.begin()) {
    lcd.print("Erro no RTC!");
    while (1); 
  }

  // Inicialização EEPROM e Leitura dos Dados
  byte eepromStatus = EEPROM.read(0);
  if (eepromStatus != EEPROM_INIT_CODE) {
    EEPROM.put(1, t1_amanhecerIni);
    EEPROM.put(3, t2_amanhecerFim);
    EEPROM.put(5, t3_diaProporcional);
    EEPROM.put(7, t4_tarde100);
    EEPROM.put(9, t5_anoitecerIni);
    EEPROM.put(11, t6_anoitecerFim);
    
    EEPROM.write(EEPROM_ADDR_LDR, 1); 
    EEPROM.write(0, EEPROM_INIT_CODE); 
  } else {
    EEPROM.get(1, t1_amanhecerIni);
    EEPROM.get(3, t2_amanhecerFim);
    EEPROM.get(5, t3_diaProporcional);
    EEPROM.get(7, t4_tarde100);
    EEPROM.get(9, t5_anoitecerIni);
    EEPROM.get(11, t6_anoitecerFim);
    
    sensorLuzAtivo = EEPROM.read(EEPROM_ADDR_LDR) == 1; 
  }

  digitalWrite(PINO_VALVULA, LOW); 
  valvulaLigada = false;
  analogWrite(PINO_DIMMER, 0);     
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();

  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset(); 
  
  if (modoMenu == 0) agora = rtc.now(); 

  // --- OVERRIDE MANUAL: LIGAR/DESLIGAR SENSOR DE LUZ (LDR) ---
  bool btnUpCru = (digitalRead(BTN_UP) == LOW);
  bool btnDownCru = (digitalRead(BTN_DOWN) == LOW);

  if (btnUpCru && btnDownCru && modoMenu == 0) {
    if (!comboPressionado) {
      comboPressionado = true;
      tempoBotoesPressione = millis();
    } 
    else if (millis() - tempoBotoesPressione >= 1000) {
      sensorLuzAtivo = !sensorLuzAtivo; 
      EEPROM.update(EEPROM_ADDR_LDR, sensorLuzAtivo ? 1 : 0);
      
      lcd.clear();
      lcd.backlight();
      if (sensorLuzAtivo) lcd.print("SENSOR DE LUZ ON"); 
      else lcd.print("SENSOR DE LUZ OF"); 
      
      delay(2000); 
      lcd.clear();
      
      while(digitalRead(BTN_UP) == LOW || digitalRead(BTN_DOWN) == LOW) {
        wdt_reset(); 
        delay(10);
      }
      comboPressionado = false; 
    }
  } else {
    comboPressionado = false; 
  }

  processarDebounceBotoes();
  gerenciarMenuAjuste();

  // --- MÁQUINA DE ESTADOS DO DISPLAY E REARME ---
  static unsigned long tempoSegurandoUP = 0;

  if (erroBombaTravada) {
    telaErroBomba();
    
    // Lógica para rearmar segurando o botão UP por 5 segundos
    if (digitalRead(BTN_UP) == LOW) {
      if (tempoSegurandoUP == 0) tempoSegurandoUP = millis();
      if (millis() - tempoSegurandoUP >= 5000) {
        erroBombaTravada = false; // Destrava o sistema
        tempoSegurandoUP = 0;
        lcd.clear();
        lcd.print(" BOMBA REARMADA ");
        delay(2000);
        lcd.clear();
      }
    } else {
      tempoSegurandoUP = 0; // Zera o cronômetro se soltar o dedo antes
    }
  } 
  else if (modoMenu > 0) {
    telaAjusteHora(); 
  } 
  else {
    if (telaTemporariaAtiva > 0) {
      if (millis() - tempoExibicaoTela > 3000) {
        telaTemporariaAtiva = 0; 
        lcd.clear(); 
      } else {
        if (telaTemporariaAtiva == 1) telaNivelAgua();
        else if (telaTemporariaAtiva == 2) telaNivelGama();
      }
    } else {
      telaPrincipal(); 
    }
  }

  // --- CICLO DA LUZ E DISPLAY (A cada 1 Segundo) ---
  if (millis() - ultimoCicloLuz >= 1000) {
    ultimoCicloLuz += 1000; // Correção para evitar drift de tempo
    
    if (modoMenu == 0) {
      controlarLuz();
    }
  }

  // --- CICLO DO ULTRASSÔNICO E BOMBA (A cada 5 Segundos) ---
  if (millis() - ultimoCicloUltrassom >= 5000) { 
    ultimoCicloUltrassom = millis(); 
    
    atualizarBarraLEDsCFTV(); // Faz a leitura pesada e manda via I2C
    
    if (modoMenu == 0) {
      controlarNivelAgua();   // Avalia a histerese da válvula
    }
  }
}

// --- CONTROLE DE NÍVEL DE ÁGUA (HISTERESE) ---
void controlarNivelAgua() {
  if (porcentagemAguaGlobal <= 10 && !valvulaLigada) {
    valvulaLigada = true; 
    tempoBombaLigada = millis(); // Marca a hora exata da partida
    digitalWrite(PINO_VALVULA, HIGH);
  } 
  else if (porcentagemAguaGlobal >= 100 && valvulaLigada) {
    valvulaLigada = false;
    digitalWrite(PINO_VALVULA, LOW);  
  }
  // FAIL-SAFE: Desliga na marra se estourar o tempo limite (ex: cano quebrado ou sensor sujo)
  if (valvulaLigada && (millis() - tempoBombaLigada > TEMPO_MAXIMO_BOMBA)) {
    valvulaLigada = false;
    digitalWrite(PINO_VALVULA, LOW);
    erroBombaTravada = true; // Aciona o estado de falha global
  }
}

// --- FOTOPERÍODO E GAMA ---
void controlarLuz() {
  long minutosAtuais = agora.hour() * 60 + agora.minute();
  long segundosDoDia = minutosAtuais * 60 + agora.second();
  int indiceLinearPWM = 0; 

  if (minutosAtuais >= t1_amanhecerIni && minutosAtuais < t2_amanhecerFim) {
    long segundosPassados = segundosDoDia - (t1_amanhecerIni * 60L);
    long segundosTotaisRampa = (t2_amanhecerFim - t1_amanhecerIni) * 60L; 
    indiceLinearPWM = map(segundosPassados, 0, segundosTotaisRampa, 0, 255);
  }
  else if (minutosAtuais >= t2_amanhecerFim && minutosAtuais < t3_diaProporcional) {
    indiceLinearPWM = 255;
  }
  else if (minutosAtuais >= t3_diaProporcional && minutosAtuais < t4_tarde100) {
    if (sensorLuzAtivo) {
      static float pwmIntegrativo = 255; 
      static unsigned long tempoUltimaAcao = millis();

      const int ALVO_LDR = 400;   
      const int JANELA = 80;      

      if (millis() - tempoUltimaAcao >= 100) { 
        int leituraLDR = analogRead(PINO_LDR);
        
        if (leituraLDR < (ALVO_LDR - JANELA)) pwmIntegrativo += 0.5; 
        else if (leituraLDR > (ALVO_LDR + JANELA)) pwmIntegrativo -= 0.5; 

        if (pwmIntegrativo > 255) pwmIntegrativo = 255;
        if (pwmIntegrativo < 0) pwmIntegrativo = 0;
        
        tempoUltimaAcao = millis();
      }
      indiceLinearPWM = (int)pwmIntegrativo;
    } 
    else {
      indiceLinearPWM = 0; 
    }
  }
  else if (minutosAtuais >= t4_tarde100 && minutosAtuais < t5_anoitecerIni) {
    indiceLinearPWM = 255;
  }
  else if (minutosAtuais >= t5_anoitecerIni && minutosAtuais < t6_anoitecerFim) {
    long segundosPassados = segundosDoDia - (t5_anoitecerIni * 60L);
    long segundosTotaisRampa = (t6_anoitecerFim - t5_anoitecerIni) * 60L;
    indiceLinearPWM = map(segundosPassados, 0, segundosTotaisRampa, 255, 0); 
  }
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
    
    long duracao = pulseIn(PINO_ECHO, HIGH, 30000); 
    if(duracao == 0) duracao = 30000; 
    
    soma += (duracao * 0.034 / 2);
    delay(10); 
  }
  return soma / 3;
}

void atualizarBarraLEDsCFTV() {
  int dist = lerUltrassonicoEstavel();
  porcentagemAguaGlobal = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagemAguaGlobal = constrain(porcentagemAguaGlobal, 0, 100);
  
  // Converte a porcentagem da água no preenchimento da Barra de LEDs
  byte estadoLeds = 0;
  if (porcentagemAguaGlobal > 20) estadoLeds |= 0b00000001; 
  if (porcentagemAguaGlobal > 30) estadoLeds |= 0b00000011;
  if (porcentagemAguaGlobal > 40) estadoLeds |= 0b00000111;
  if (porcentagemAguaGlobal > 50) estadoLeds |= 0b00001111;
  if (porcentagemAguaGlobal > 60) estadoLeds |= 0b00011111;
  if (porcentagemAguaGlobal > 70) estadoLeds |= 0b00111111;
  if (porcentagemAguaGlobal > 80) estadoLeds |= 0b01111111;
  if (porcentagemAguaGlobal > 90) estadoLeds |= 0b11111111;

  Wire.beginTransmission(ENDERECO_PCF_LEDS);
  Wire.write(~estadoLeds); 
  Wire.endTransmission();
}

// --- BOTÕES E MENU INTELIGENTE ---
void processarDebounceBotoes() {
  static unsigned long tempoUltimaMudancaMENU = 0;
  static unsigned long tempoUltimaMudancaUP = 0;
  static unsigned long tempoUltimaMudancaDOWN = 0;
  static bool ultimoEstadoCruMENU = HIGH;
  static bool ultimoEstadoCruUP = HIGH;
  static bool ultimoEstadoCruDOWN = HIGH;

  menuFoiClicado = false;
  upFoiClicado = false;
  downFoiClicado = false;

  bool leituraAtualMENU = digitalRead(BTN_MENU); 
  if (leituraAtualMENU != ultimoEstadoCruMENU) tempoUltimaMudancaMENU = millis(); 
  if ((millis() - tempoUltimaMudancaMENU) > tempoDebounce) {
    if (leituraAtualMENU != menuPressionado) {
      menuPressionado = leituraAtualMENU; 
      if (menuPressionado == LOW) menuFoiClicado = true;
    }
  }
  ultimoEstadoCruMENU = leituraAtualMENU;

  bool leituraAtualUP = digitalRead(BTN_UP);
  if (leituraAtualUP != ultimoEstadoCruUP) tempoUltimaMudancaUP = millis();
  if ((millis() - tempoUltimaMudancaUP) > tempoDebounce) {
    if (leituraAtualUP != upPressionado) {
      upPressionado = leituraAtualUP;
      if (upPressionado == LOW) upFoiClicado = true;
    }
  }
  ultimoEstadoCruUP = leituraAtualUP;

  bool leituraAtualDOWN = digitalRead(BTN_DOWN);
  if (leituraAtualDOWN != ultimoEstadoCruDOWN) tempoUltimaMudancaDOWN = millis();
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
  if (variavelTempo < 0) variavelTempo = 1425;
}

void gerenciarMenuAjuste() {
  static unsigned long tempoInicioSegurarMenu = 0;
  static bool salvamentoExecutado = false;
  static bool ignorarSoltura = false; 

  if (modoMenu == 0) {
    if (menuPressionado == LOW) {
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      if ((millis() - tempoInicioSegurarMenu) > 1000) {
        lcd.clear();
        modoMenu = 1;
        horaAjuste = agora.hour();
        minutoAjuste = agora.minute();
        tempoInicioSegurarMenu = 0;
        ignorarSoltura = true; 
      }
    } else {
      tempoInicioSegurarMenu = 0;
    }

    if (upFoiClicado && !comboPressionado) {
      telaTemporariaAtiva = 1; 
      tempoExibicaoTela = millis();
      lcd.clear();
    }
    if (downFoiClicado && !comboPressionado) {
      telaTemporariaAtiva = 2; 
      tempoExibicaoTela = millis();
      lcd.clear();
    }
  } 
  else {
    if (menuPressionado == LOW) {
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      
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
        delay(2000);
        
        modoMenu = 0;
        tempoInicioSegurarMenu = 0; 
        menuFoiClicado = false;     
        lcd.clear();
        return;                     
      }
    } 
    else { 
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
            delay(2000);
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
  if (valvulaLigada) lcd.print("ENCHENDO A CAIXA"); 
  else lcd.print("NIVEL DA CAIXA: "); 
  
  lcd.setCursor(0, 1);
  lcd.write(0); 
  lcd.print(" ");
  if(porcentagemAguaGlobal < 100) lcd.print(" "); 
  if(porcentagemAguaGlobal < 10) lcd.print(" ");  
  lcd.print(porcentagemAguaGlobal);
  lcd.print("%          "); 
}

void telaErroBomba() {
  lcd.setCursor(0, 0);
  bool pisca = ((millis() / 2000) % 2 == 0);
  
  if (pisca) {
    lcd.print("TEMPO EXCEDIDO!!");
  } else {
    lcd.print("BOMBA DESLIGADA!");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("SEGURE UP (5s)  ");
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
