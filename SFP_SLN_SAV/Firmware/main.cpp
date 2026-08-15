/* * SIMULADOR DE FOTOPERIODO - SFP_SLN_SAV - VERSÃO 9.1
 * Hardware: ATmega328P, DS3231, LCD I2C, LDR, HC-SR04, Relé, IRF3205.
 * Detecção de nível com ultrassonico e acionamento da bomba que enche o reservatório via relé.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <avr/wdt.h> // Watchdog
#include <EEPROM.h>  // Memória Não Volátil

// --- LISTA DE FUNCOES ---
void processarDebounceBotoes();
void gerenciarMenuAjuste();
void telaErroBomba();
void telaAjusteHora();
void telaNivelAgua();
void telaNivelGama();
void telaPrincipal();
void controlarLuz();
void atualizarBarraLEDsCFTV();
void controlarNivelAgua();
void aplicarPWMSeguro(uint16_t pwmValue); 
void executarSalvamento();
bool validarAvanco(int telaAtual);

// --- MAPEAMENTO DE PINOS ---
#define PINO_LDR        A0    // Pino para o LDR
#define BTN_MENU        A1    // Botão menu
#define BTN_UP          A2    // Botão UP 
#define BTN_DOWN        A3    // Botão DOWN
#define PINO_DIMMER     9     // Pino do PWM
#define PINO_VALVULA    3     // Pino de ligar a bomba
#define PINO_TRIGGER    10    // Pino do Sensor de Nível
#define PINO_ECHO       11    // Pino do sensor de nível

// Endereço I2C do Módulo PCF8574 extra para LEDs
#define ENDERECO_PCF_LEDS 0x26 

// --- CONFIGURAÇÕES DO SISTEMA ---
const int DISTANCIA_TANQUE_VAZIO = 30; // cm (Ajustar em campo)
const int DISTANCIA_TANQUE_CHEIO = 14;  // cm (Ajustar em campo)
const unsigned long tempoDebounce = 50; 

// --- VARIÁVEIS DE SEGURANÇA TÉRMICA ---
unsigned long ultimoCicloTemp = 0;
bool erroSuperaquecimento = false;
const float TEMP_CRITICA = 55.0;     // Temperatura de corte
const float TEMP_RECUPERACAO = 45.0; // Temperatura segura para rearmar a luz

#define EEPROM_INIT_CODE 0x42 
#define EEPROM_ADDR_LDR  13 // Endereço para salvar o status do LDR
#define EEPROM_ADDR_ERRO_BOMBA 14 // Endereço para salvar se a bomba travou, para que se houver um reboot durante o erro, o sistema continuar travado evitando que a bomba ligue

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
const unsigned long TEMPO_MAXIMO_BOMBA = 420000; // 7 minutos em milissegundos (ajustar em campo)
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

//=================================================================================================

void setup() {
  pinMode(PINO_DIMMER, OUTPUT);

// Limpeza dos registradores
  TCCR1A = 0;
  TCCR1B = 0;

// PWM, Modo 14 (TOP = ICR1) | Saída não-invertida no pino 9 (COM1A1)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // Prescaler = 1 (Frequência máxima)
  ICR1 = 32767; // Define o teto (TOP) gerando 488 Hz exatos
  OCR1A = 0;    // Inicia com ciclo de trabalho em 0% (Luz apagada)

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

//=================================================================================================

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
    EEPROM.write(EEPROM_ADDR_ERRO_BOMBA, 0); // 0 = Sem erro de bomba
    EEPROM.write(0, EEPROM_INIT_CODE); 
  } else {
    EEPROM.get(1, t1_amanhecerIni);
    EEPROM.get(3, t2_amanhecerFim);
    EEPROM.get(5, t3_diaProporcional);
    EEPROM.get(7, t4_tarde100);
    EEPROM.get(9, t5_anoitecerIni);
    EEPROM.get(11, t6_anoitecerFim);
    
    sensorLuzAtivo = EEPROM.read(EEPROM_ADDR_LDR) == 1;
    erroBombaTravada = EEPROM.read(EEPROM_ADDR_ERRO_BOMBA) == 1; 
  }

  digitalWrite(PINO_VALVULA, LOW); 
  valvulaLigada = false;
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();

  wdt_enable(WDTO_8S);
}

//============================================================================================================

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

//=====================================================================================

  // --- MÁQUINA DE ESTADOS DO DISPLAY E REARME ---
  static unsigned long tempoSegurandoUP = 0;

  if (erroSuperaquecimento) {
    // Alarme Visual de Calor (Prioridade 1)
    lcd.setCursor(0, 0);
    lcd.print("Superaquecimento");
    lcd.setCursor(0, 1);
    lcd.print("Luz Desligada!!!");
    modoMenu = 0; // Força a saída de qualquer menu ativo
  }
  else if (erroBombaTravada) {
    telaErroBomba(); // Esconde a tela principal (Prioridade 2)
    
    // Lógica para rearmar segurando o botão UP por 5 segundos
    if (digitalRead(BTN_UP) == LOW) {
      if (tempoSegurandoUP == 0) tempoSegurandoUP = millis();
      if (millis() - tempoSegurandoUP >= 5000) {
        erroBombaTravada = false; // Destrava o sistema na RAM
        EEPROM.update(EEPROM_ADDR_ERRO_BOMBA, 0); // Destrava o sistema na EEPROM
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
    controlarNivelAgua();   // Avalia a histerese da válvula
}

// --- CICLO TÉRMICO (A cada 30 Segundos) ---
  if (millis() - ultimoCicloTemp >= 30000) { 
    ultimoCicloTemp = millis(); 
    
    float tempAtual = rtc.getTemperature(); 
    
    // Trava e destrava com histerese
    if (tempAtual >= TEMP_CRITICA) {
      erroSuperaquecimento = true;
    } 
    else if (tempAtual <= TEMP_RECUPERACAO) {
      erroSuperaquecimento = false;
    }
  }
}

//======================================================================================

// --- CONTROLE DE NÍVEL DE ÁGUA (HISTERESE) ---
void controlarNivelAgua() {
  // Intertravamento: A bomba SÓ pode ligar se NÃO houver erro travado na memória
  if (porcentagemAguaGlobal <= 10 && !valvulaLigada && !erroBombaTravada) {
    valvulaLigada = true; 
    tempoBombaLigada = millis(); // Marca a hora exata da partida
    digitalWrite(PINO_VALVULA, HIGH);
  } 
  else if (porcentagemAguaGlobal >= 100 && valvulaLigada) {
    valvulaLigada = false;
    digitalWrite(PINO_VALVULA, LOW);  
  }
  
  // FAIL-SAFE: Desliga na marra se estourar o tempo limite
  if (valvulaLigada && (millis() - tempoBombaLigada > TEMPO_MAXIMO_BOMBA)) {
    valvulaLigada = false;
    digitalWrite(PINO_VALVULA, LOW);
    
    // Trava o sistema e salva na memória não-volátil (apenas se mudou de estado)
    if (!erroBombaTravada) {
      erroBombaTravada = true; 
      EEPROM.update(EEPROM_ADDR_ERRO_BOMBA, 1); 
      modoMenu = 0; // Força a saída do Menu se estourar o tempo de enchimento da caixa
    }
  }
}

//======================================================================================

// --- FOTOPERIODO E GAMA ---
void controlarLuz() {
  long minutosAtuais = agora.hour() * 60 + agora.minute();
  long segundosDoDia = minutosAtuais * 60 + agora.second();
  
  float progresso = 0.0; 
  uint16_t pwmAlvo = 0;

  if (minutosAtuais >= t1_amanhecerIni && minutosAtuais < t2_amanhecerFim) {
    long segPassados = segundosDoDia - (t1_amanhecerIni * 60L);
    long segTotais = (t2_amanhecerFim - t1_amanhecerIni) * 60L;
    progresso = (float)segPassados / segTotais; // Vai de 0.0 a 1.0 suavemente
    // Calcula a curva Gama em tempo real e joga pra escala de 15 bits
    pwmAlvo = (uint16_t)(pow(progresso, 2.5) * 32767.0);
  }
  else if (minutosAtuais >= t2_amanhecerFim && minutosAtuais < t3_diaProporcional) {
    pwmAlvo = 32767;
  }
else if (minutosAtuais >= t3_diaProporcional && minutosAtuais < t4_tarde100) {
    if (sensorLuzAtivo) {
       int leituraLDR = analogRead(PINO_LDR);
       
       // Mapeamento dinamico: Converte a leitura analogica (0 a 1023) para o PWM de 15 bits (0 a 32767).
       // Se ambiente ESCURO = leitura ALTA (ex: 1000), o map abaixo deixará a luz 100% no escuro.
       long pwmMapeado = map(leituraLDR, 2, 993, 32767, 0); 
       
       pwmAlvo = constrain(pwmMapeado, 0, 32767); // Trava de segurança para não estourar a variável
    } else {
       // Logica manual: Se desligar o sensor pressionando UP+DOWN, a luz desliga
       // Se quiser que ela fique 100% ligada ignorando o sol, basta trocar o 0 abaixo por 32767.
       pwmAlvo = 0; 
    }
  }
  else if (minutosAtuais >= t4_tarde100 && minutosAtuais < t5_anoitecerIni) {
    pwmAlvo = 32767;
  }
  else if (minutosAtuais >= t5_anoitecerIni && minutosAtuais < t6_anoitecerFim) {
    long segPassados = segundosDoDia - (t5_anoitecerIni * 60L);
    long segTotais = (t6_anoitecerFim - t5_anoitecerIni) * 60L;
    progresso = 1.0 - ((float)segPassados / segTotais); // Decresce de 1.0 a 0.0
    pwmAlvo = (uint16_t)(pow(progresso, 2.5) * 32767.0);
  }

  // INTERTRAVAMENTO TERMICO
  if (erroSuperaquecimento) pwmAlvo = 0;

  // PISO DE SOFTWARE para o TLP250
  // 1 tick = 62,5ns. TLP250 precisa de 500ns = 8 ticks.
  // Se o PWM for maior que zero, mas nao tiver força pra ligar o driver, pula pro 8.
  if (pwmAlvo > 0 && pwmAlvo < 8) pwmAlvo = 8; 

  aplicarPWMSeguro(pwmAlvo); 
  estadoLuz = (pwmAlvo > 0);
  pwmAtualGlobal = pwmAlvo;
}

//========================================================================================

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

//===========================================================================================

void atualizarBarraLEDsCFTV() {
  int dist = lerUltrassonicoEstavel();
  porcentagemAguaGlobal = map(dist, DISTANCIA_TANQUE_VAZIO, DISTANCIA_TANQUE_CHEIO, 0, 100);
  porcentagemAguaGlobal = constrain(porcentagemAguaGlobal, 0, 100);
  
  // Converte a porcentagem da agua no preenchimento da Barra de LEDs
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
  Wire.write(~estadoLeds); //Anodo comum
  Wire.endTransmission();
}

//==========================================================================================

// --- BOTOES E MENU INTELIGENTE ---
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

//=========================================================================================================================================

void ajustarVariavelTempo(int &variavelTempo, bool aumenta) {
  if (aumenta) variavelTempo += 5;
  else variavelTempo -= 5;
  if (variavelTempo > 1439) variavelTempo = 0;
  if (variavelTempo < 0) variavelTempo = 1425;
}

//=========================================================================================================================================
void gerenciarMenuAjuste() {
  if (erroSuperaquecimento) return; 
  if (erroBombaTravada) return; 
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
        
        // LOGICA DE SALVAMENTO RAPIDO
        bool sucesso = true;
        while (modoMenu <= 8) {
          if (!validarAvanco(modoMenu)) {
            sucesso = false;
            break; // Acha o erro, quebra o loop e leva exatamente pra tela problematica
          }
          modoMenu++;
        }

        if (sucesso) { // Passou pela varredura em todas as telas ileso
          executarSalvamento();
          modoMenu = 0;
        }
        
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
        
        // LOGICA DE AVANCO PASSO A PASSO
        if (duracao < 1000 && !salvamentoExecutado) {
          if (validarAvanco(modoMenu)) { // So avanca se a tela atual estiver certa
            modoMenu++;
            if (modoMenu > 8) {
              executarSalvamento();
              modoMenu = 0; 
            }
          }
          lcd.clear(); 
        }
      }
      tempoInicioSegurarMenu = 0;
      salvamentoExecutado = false;
    }

    // LEITURA UP/DOWN NAS TELAS
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
//=========================================================================================================================================

// --- FUNCOES DE EXIBICAO NO LCD ---
void printHoraFormatada(int minutosTotais) {
  int h = minutosTotais / 60;
  int m = minutosTotais % 60;
  if(h < 10) lcd.print('0');
  lcd.print(h);
  lcd.print(':');
  if(m < 10) lcd.print('0');
  lcd.print(m);
}

//=========================================================================================================================================

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

//=========================================================================================================================================

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

//=========================================================================================================================================

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

//=========================================================================================================================================

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

//=========================================================================================================================================

void telaNivelGama() {
  lcd.setCursor(0, 0);
  lcd.print(" NIVEL DE LUZ:  "); 
  lcd.setCursor(0, 1);
  lcd.print("PWM:");
  
  // Atualizado para a nova escala do Timer 1 (15 bits)
  int percLuz = map(pwmAtualGlobal, 0, 32767, 0, 100);
  
  if (percLuz < 100) lcd.print(" "); 
  if (percLuz < 10) lcd.print(" ");  
  lcd.print(percLuz);
  lcd.print("% ");
  
  // Formatacao dinamica para caber numeros de 1 a 5 digitos sem estourar a tela
  if (pwmAtualGlobal < 10000) lcd.print(" ");
  if (pwmAtualGlobal < 1000) lcd.print(" ");
  if (pwmAtualGlobal < 100) lcd.print(" ");
  if (pwmAtualGlobal < 10) lcd.print(" ");
  lcd.print(pwmAtualGlobal); 
}

//=========================================================================================================================================

// --- FUNCAO GERENCIADORA DE PWM (SOFTSTART) ---
void aplicarPWMSeguro(uint16_t pwmAlvo) {
  static uint16_t pwmRealNaPlaca = 0; 
  if (pwmAlvo == pwmRealNaPlaca) return;

  if (pwmAlvo > pwmRealNaPlaca) {
    // Calcula um salto de aceleracao. Se pular de 0 a 100%, faz em aprox. 1.5 segundos
    uint16_t salto = (pwmAlvo - pwmRealNaPlaca) / 100;
    if (salto < 1) salto = 1;

    for (uint16_t i = pwmRealNaPlaca; i <= pwmAlvo; i += salto) {
      OCR1A = i; // Escreve direto no registrador de hardware do Pino 9
      delay(15); 
      wdt_reset(); 
      if (pwmAlvo - i < salto) break; // Trava de precisão para o ultimo loop
    }
  } 
  
  // Confirmacao final exata
  OCR1A = pwmAlvo;
  pwmRealNaPlaca = pwmAlvo; 
}

//=========================================================================================================================================

// Funcao que valida a regra antes de deixar o usuario avancar de tela
bool validarAvanco(int telaAtual) {
  switch (telaAtual) {
    case 4: // Tentando sair do Amanhecer Fim
      if (t2_amanhecerFim <= t1_amanhecerIni) {
        lcd.clear(); lcd.print("ERRO: AMANHECER!"); lcd.setCursor(0, 1); lcd.print("Fim <= Inicio   "); delay(3000);
        return false;
      }
      break;
    case 5: // Tentando sair do Ligar Sensor
      if (t3_diaProporcional < t2_amanhecerFim) {
        lcd.clear(); lcd.print("ERRO: LIGAR SENS"); lcd.setCursor(0, 1); lcd.print("Sensor < Amanhec"); delay(3000);
        return false;
      }
      break;
    case 6: // Tentando sair da Luz Tarde
      if (t4_tarde100 < t3_diaProporcional) {
        lcd.clear(); lcd.print("ERRO: LUZ TARDE "); lcd.setCursor(0, 1); lcd.print("Tarde < Sensor  "); delay(3000);
        return false;
      }
      break;
    case 7: // Tentando sair do Anoitecer Ini
      if (t5_anoitecerIni < t4_tarde100) {
        lcd.clear(); lcd.print("ERRO: ANOITECER!"); lcd.setCursor(0, 1); lcd.print("Anoitec. < Tarde"); delay(3000);
        return false;
      }
      break;
    case 8: // Tentando finalizar o ajuste saindo do Anoitecer Fim
      if (t6_anoitecerFim <= t5_anoitecerIni) {
        lcd.clear(); lcd.print("ERRO: ANOITECER!"); lcd.setCursor(0, 1); lcd.print("Fim <= Inicio   "); delay(3000);
        return false;
      }
      if (t6_anoitecerFim >= 1440) {
        lcd.clear(); lcd.print("ERRO: LIMITE DIA"); lcd.setCursor(0, 1); lcd.print("Passou das 00:00"); delay(3000);
        return false;
      }
      break;
  }
  return true; // Se a regra estiver certa (ou se for menu de relogio), permite o avanco
}

//=========================================================================================================================================

// Funcao focada apenas em gravar na EEPROM quando tudo estiver validado
void executarSalvamento() {
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
  lcd.print(" COM SUCESSO! :)");
  delay(2000);
}
