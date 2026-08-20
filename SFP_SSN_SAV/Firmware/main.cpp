/* * SIMULADOR DE FOTOPERIODO - SFP_SSN_SAV - VERSAO 1.3
 * Hardware: ATmega328P, DS3231, LCD I2C, LDR, IRF3205 (TLP250).
 * Modo automatico/manual, habilita/desabilita o fotoperiodo ao pressionar os 3 botoes simultaneamente por 1s.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <avr/wdt.h> // Watchdog
#include <EEPROM.h>  // Memoria Nao Volatil

// LISTA DE FUNCOES
void processarDebounceBotoes();
void gerenciarMenuAjuste();
void telaAjusteHora();
void telaNivelGama();
void telaPrincipal();
void controlarLuz();
void aplicarPWMSeguro(); 
void executarSalvamento();
bool validarAvanco(int telaAtual);
void esperarEAtualizarPWM(unsigned long tempoEspera);

// MAPEAMENTO DE PINOS
#define PINO_LDR        A0    // Pino para o LDR
#define BTN_MENU        A1    // Botao menu
#define BTN_UP          A2    // Botao UP 
#define BTN_DOWN        A3    // Botao DOWN
#define PINO_DIMMER     9     // Pino do PWM

// --- CONFIGURAÇÕES DO SISTEMA ---
const unsigned long tempoDebounce = 50; 

// --- VARIAVEIS DE SEGURANCA TERMICA ---
unsigned long ultimoCicloTemp = 0;
bool erroSuperaquecimento = false;
const float TEMP_CRITICA = 55.0;     // Temperatura de corte
const float TEMP_RECUPERACAO = 45.0; // Temperatura segura para rearmar a luz

#define EEPROM_INIT_CODE 0x42 
#define EEPROM_ADDR_LDR  13 // Endereco para salvar o status do LDR
#define EEPROM_ADDR_FOTO 15 // Endereco para salvar o Modo Automatico/Manual

// --- OBJETOS ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 
RTC_DS3231 rtc; 

// --- VARIAVEIS GLOBAIS ---
DateTime agora;
unsigned long ultimoCicloLuz = 0; // Cronometro para a Luz (1s)

int pwmAtualGlobal = 0; 
uint16_t pwmAlvoGlobal = 0;
bool estadoLuz = false;

// Variaveis: LDR Manual Override
bool sensorLuzAtivo = true; 
unsigned long tempoBotoesPressione = 0;
bool comboPressionado = false;
bool fotoperiodoAtivo = true; // true = Modo Automatico (Curva), false = Modo Manual
bool luzManualLigada = false; // true = 100%, false = 0% no modo manual

// Variaveis para as Telas "Pop-up" Temporarias
byte telaTemporariaAtiva = 0; 
unsigned long tempoExibicaoTela = 0;

// Variaveis do Cronograma (Armazenadas em Minutos)
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

//=================================================================================================

void setup() {
  pinMode(PINO_DIMMER, OUTPUT);

// Limpeza dos registradores
  TCCR1A = 0;
  TCCR1B = 0;

// PWM, Modo 14 (TOP = ICR1) | Saida nao-invertida no pino 9 (COM1A1)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // Prescaler = 1 (Frequencia maxima)
  ICR1 = 32767; // Define o teto (TOP) gerando 488 Hz exatos
  OCR1A = 0;    // Inicia com ciclo de trabalho em 0% (Luz apagada)

  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  Wire.begin();
  lcd.init();
  lcd.backlight();

  if (!rtc.begin()) {
    lcd.print("Erro no RTC!");
    while (1); 
  }

//=================================================================================================

  // Inicializacao EEPROM e Leitura dos Dados
  byte eepromStatus = EEPROM.read(0);
  if (eepromStatus != EEPROM_INIT_CODE) {
    EEPROM.put(1, t1_amanhecerIni);
    EEPROM.put(3, t2_amanhecerFim);
    EEPROM.put(5, t3_diaProporcional);
    EEPROM.put(7, t4_tarde100);
    EEPROM.put(9, t5_anoitecerIni);
    EEPROM.put(11, t6_anoitecerFim);
    
    EEPROM.write(EEPROM_ADDR_LDR, 1); 
    EEPROM.write(EEPROM_ADDR_FOTO, 1);
    EEPROM.write(0, EEPROM_INIT_CODE); 
  } else {
    EEPROM.get(1, t1_amanhecerIni);
    EEPROM.get(3, t2_amanhecerFim);
    EEPROM.get(5, t3_diaProporcional);
    EEPROM.get(7, t4_tarde100);
    EEPROM.get(9, t5_anoitecerIni);
    EEPROM.get(11, t6_anoitecerFim);
    
    sensorLuzAtivo = EEPROM.read(EEPROM_ADDR_LDR) == 1;
    fotoperiodoAtivo = EEPROM.read(EEPROM_ADDR_FOTO) == 1;
  }
  
  lcd.setCursor(0,0);
  lcd.print("SISTEMA INICIADO");
  delay(1000);
  lcd.clear();

  wdt_enable(WDTO_8S);
}

//============================================================================================================

void loop() {
  wdt_reset(); 
  
  aplicarPWMSeguro();
  
  if (modoMenu == 0) agora = rtc.now(); 

// --- OVERRIDE MANUAL E MODO AUTO/MANUAL ---
  bool btnMenuCru = (digitalRead(BTN_MENU) == LOW);
  bool btnUpCru = (digitalRead(BTN_UP) == LOW);
  bool btnDownCru = (digitalRead(BTN_DOWN) == LOW);

  if (btnUpCru && btnDownCru && modoMenu == 0) {
    if (!comboPressionado) {
      comboPressionado = true;
      tempoBotoesPressione = millis();
    } 
    else if (millis() - tempoBotoesPressione >= 1000) {
      lcd.clear();
      lcd.backlight();
      
      // Se os 3 botoes estiverem pressionados: Alterna Automatico/Manual
      if (btnMenuCru) {
        fotoperiodoAtivo = !fotoperiodoAtivo; 
        EEPROM.update(EEPROM_ADDR_FOTO, fotoperiodoAtivo ? 1 : 0);
        
        lcd.print("  FOTOPERIODO:  "); 
        lcd.setCursor(0, 1);
        lcd.print(fotoperiodoAtivo ? "AUTOMATICO (ON) " : " MODO MANUAL!   "); 
      } 
      // Se apenas UP e DOWN estiverem pressionados:
      else {
        if (fotoperiodoAtivo) {
          // Comportamento normal: Liga/Desliga LDR
          sensorLuzAtivo = !sensorLuzAtivo; 
          EEPROM.update(EEPROM_ADDR_LDR, sensorLuzAtivo ? 1 : 0);
          lcd.print(sensorLuzAtivo ? "SENSOR DE LUZ ON" : "SENSOR DE LUZ OF"); 
        } else {
          // Comportamento Manual: Liga/Desliga Luz Direto
          luzManualLigada = !luzManualLigada;
          lcd.print(" LUZ MANUAL 100%");
          lcd.setCursor(0, 1);
          lcd.print(luzManualLigada ? "   -> LIGADA    " : "   -> DESLIGADA ");
        }
      }
      
      esperarEAtualizarPWM(2000);
      lcd.clear();
      
      // Trava de seguranca para esperar a soltara dos botoes
      while(digitalRead(BTN_UP) == LOW || digitalRead(BTN_DOWN) == LOW || digitalRead(BTN_MENU) == LOW) { 
      esperarEAtualizarPWM(10);
      }
      comboPressionado = false; 
    }
  } else {
    comboPressionado = false; 
  }

  processarDebounceBotoes();
  gerenciarMenuAjuste();

//=====================================================================================

  // --- MAQUINA DE ESTADOS DO DISPLAY ---
  
  if (erroSuperaquecimento) {
    // Alarme Visual de Calor (Prioridade 1)
    lcd.setCursor(0, 0);
    lcd.print("Superaquecimento");
    lcd.setCursor(0, 1);
    lcd.print("Luz Desligada!!!");
    modoMenu = 0; // Forca a saada de qualquer menu ativo
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
        if (telaTemporariaAtiva == 1) telaNivelGama();
      }
    } else {
      telaPrincipal(); 
    }
  }
  
   // --- CICLO DA LUZ E DISPLAY (A cada 1 Segundo) ---
  if (millis() - ultimoCicloLuz >= 1000) {
    ultimoCicloLuz += 1000; // Correcao para evitar drift de tempo
    
   if (modoMenu == 0) {
      controlarLuz();
    }
  }

// --- CICLO TERMICO (A cada 30 Segundos) ---
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

// --- FOTOPERIODO E GAMA ---
void controlarLuz() {

  if (!fotoperiodoAtivo) {  // --- SE MODO MANUAL ATIVO, IGNORA O RELOGIO INTEIRO ---
    pwmAlvoGlobal = luzManualLigada ? 32767 : 0;
    
    if (erroSuperaquecimento) pwmAlvoGlobal = 0; // Protecao termica continua valendo
    if (pwmAlvoGlobal > 0 && pwmAlvoGlobal < 8) pwmAlvoGlobal = 8;
    return; // Sai da funcao imediatamente, ignorando as contas abaixo
  }

  long minutosAtuais = agora.hour() * 60 + agora.minute();
  long segundosDoDia = minutosAtuais * 60 + agora.second();
  
  float progresso = 0.0; 
  uint16_t pwmAlvo = 0;

  if (minutosAtuais >= t1_amanhecerIni && minutosAtuais < t2_amanhecerFim) {
    long segPassados = segundosDoDia - (t1_amanhecerIni * 60L);
    long segTotais = (t2_amanhecerFim - t1_amanhecerIni) * 60L;
    progresso = (float)segPassados / segTotais; // Vai de 0.0 a 1.0 suavemente
    pwmAlvo = (uint16_t)(pow(progresso, 2.5) * 32767.0);  // Calcula a curva Gama em tempo real e joga pra escala de 15 bits
  }

  else if (minutosAtuais >= t2_amanhecerFim && minutosAtuais < t3_diaProporcional) {
    pwmAlvo = 32767;
  }

else if (minutosAtuais >= t3_diaProporcional && minutosAtuais < t4_tarde100) {
    if (sensorLuzAtivo) {
      static long integradorLinear = 32767; 
     // static unsigned long tempoUltimaAcao = millis();

      const int ALVO_LDR = 400;   
      const int JANELA = 80;      

    // if (millis() - tempoUltimaAcao >= 100) { 
        int leituraLDR = analogRead(PINO_LDR);
        
        if (leituraLDR < (ALVO_LDR - JANELA)) integradorLinear += 64; 
        else if (leituraLDR > (ALVO_LDR + JANELA)) integradorLinear -= 64; 

        integradorLinear = constrain(integradorLinear, 0, 32767);
      //  tempoUltimaAcao = millis();
    //  }

      // Converte a malha nativa para a proporcao da Gama (0.0 a 1.0)
      float progresso = integradorLinear / 32767.0;
      
      // Aplica a curva para garantir que a correcao pareca linear aos olhos
      pwmAlvo = (uint16_t)(pow(progresso, 2.5) * 32767.0);
    } 
    else {
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
  if (pwmAlvo > 0 && pwmAlvo < 8) pwmAlvo = 8; 
  pwmAlvoGlobal = pwmAlvo;
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
  static unsigned long tempoInicioSegurarMenu = 0;
  static bool salvamentoExecutado = false;
  static bool ignorarSoltura = false; 

  if (modoMenu == 0) {
    if (menuPressionado == LOW && !comboPressionado) {
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

    if (downFoiClicado && !comboPressionado) {
      telaTemporariaAtiva = 1; 
      tempoExibicaoTela = millis();
      lcd.clear();
    }
  } 
  else {
    if (menuPressionado == LOW) {
      if (tempoInicioSegurarMenu == 0) tempoInicioSegurarMenu = millis();
      
      if ((millis() - tempoInicioSegurarMenu) > 1000 && !salvamentoExecutado && !ignorarSoltura) {
        salvamentoExecutado = true;
 
       if (!fotoperiodoAtivo) {
          // Se for Modo Manual: Salva a hora direto e sai. Pula todas as validacoes
           executarSalvamento();
           modoMenu = 0;
         }
        else {
         // Se for Modo Automatico: Faz a varredura normal nas 8 telas
        // LOGICA DE SALVAMENTO RAPIDO
        bool sucesso = true;
        while (modoMenu <= 8) {
            if (!validarAvanco(modoMenu)) {
               sucesso = false;
               break; // Acha o erro, quebra o loop e leva exatamente pra tela problematica
               }
            modoMenu++;
            }

            if (sucesso) { // Passou ileso pela varredura em todas as telas
               executarSalvamento();
               modoMenu = 0;
               }
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
               if ((!fotoperiodoAtivo && modoMenu > 2) || (modoMenu > 8)) {
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

   if (!fotoperiodoAtivo) {
       lcd.print("<<<MODO MANUAL>>>");
   }
   else {
   
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
}

//=========================================================================================================================================

void telaNivelGama() {
  lcd.setCursor(0, 0);

  if (!fotoperiodoAtivo){
    lcd.print("  MODO MANUAL:  "); 
    lcd.setCursor(0, 1);
    lcd.print(luzManualLigada ? "   LUZ LIGADA   " : " LUZ DESLIGADA  ");
   }
   else {
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
 }

//=========================================================================================================================================

// MAQUINA DE ESTADOS DO PWM (ASSINCRONA E NAO-BLOQUEANTE) ---
void aplicarPWMSeguro() {
  static uint16_t pwmRealNaPlaca = 0; 
  static unsigned long tempoUltimoPasso = 0;

  // Se o brilho fisico ja chegou na meta logica, sai da funcao instantaneamente
  if (pwmRealNaPlaca == pwmAlvoGlobal) return; 

  // Executa 1 micro-passo a cada 2 milissegundos (SEM NENHUM DELAY)
  if (millis() - tempoUltimoPasso >= 2) {
    tempoUltimoPasso = millis();

    // Calcula a distancia ate o alvo
    uint16_t diferenca = pwmAlvoGlobal > pwmRealNaPlaca ? (pwmAlvoGlobal - pwmRealNaPlaca) : (pwmRealNaPlaca - pwmAlvoGlobal);
    
    // Aceleracao adaptativa: saltos maiores se a diferenca for grande, saltos precisos perto do fim
    uint16_t salto = diferenca / 50; 
    if (salto < 15) salto = 15; // Piso minimo para a rampa nao ficar lenta demais

    // Rampa de Subida
    if (pwmRealNaPlaca < pwmAlvoGlobal) {
      pwmRealNaPlaca += salto;
      if (pwmRealNaPlaca > pwmAlvoGlobal) pwmRealNaPlaca = pwmAlvoGlobal; // Trava no limite
    } 
    // Rampa de Descida
    else {
      if (pwmRealNaPlaca > salto) pwmRealNaPlaca -= salto;
      else pwmRealNaPlaca = 0;
      if (pwmRealNaPlaca < pwmAlvoGlobal) pwmRealNaPlaca = pwmAlvoGlobal; // Trava no limite
    }

    // Atualiza o registrador de hardware e as variaveis do display
    OCR1A = pwmRealNaPlaca;
    pwmAtualGlobal = pwmRealNaPlaca; 
    estadoLuz = (pwmRealNaPlaca > 0);
  }
}
//=========================================================================================================================================

// Funcao que valida a regra antes de deixar o usuario avancar de tela
bool validarAvanco(int telaAtual) {
  switch (telaAtual) {
    case 4: // Tentando sair do Amanhecer Fim
      if (t2_amanhecerFim <= t1_amanhecerIni) {
        lcd.clear(); lcd.print("ERRO: AMANHECER!"); lcd.setCursor(0, 1); lcd.print("Fim <= Inicio   "); esperarEAtualizarPWM(3000);
        return false;
      }
      break;
    case 5: // Tentando sair do Ligar Sensor
      if (t3_diaProporcional < t2_amanhecerFim) {
        lcd.clear(); lcd.print("ERRO: LIGAR SENS"); lcd.setCursor(0, 1); lcd.print("Sensor < Amanhec"); esperarEAtualizarPWM(3000);
        return false;
      }
      break;
    case 6: // Tentando sair da Luz Tarde
      if (t4_tarde100 < t3_diaProporcional) {
        lcd.clear(); lcd.print("ERRO: LUZ TARDE "); lcd.setCursor(0, 1); lcd.print("Tarde < Sensor  "); esperarEAtualizarPWM(3000);
        return false;
      }
      break;
    case 7: // Tentando sair do Anoitecer Ini
      if (t5_anoitecerIni < t4_tarde100) {
        lcd.clear(); lcd.print("ERRO: ANOITECER!"); lcd.setCursor(0, 1); lcd.print("Anoitec. < Tarde"); esperarEAtualizarPWM(3000);
        return false;
      }
      break;
    case 8: // Tentando finalizar o ajuste saindo do Anoitecer Fim
      if (t6_anoitecerFim <= t5_anoitecerIni) {
        lcd.clear(); lcd.print("ERRO: ANOITECER!"); lcd.setCursor(0, 1); lcd.print("Fim <= Inicio   "); esperarEAtualizarPWM(3000);
        return false;
      }
      if (t6_anoitecerFim >= 1440) {
        lcd.clear(); lcd.print("ERRO: LIMITE DIA"); lcd.setCursor(0, 1); lcd.print("Passou das 00:00"); esperarEAtualizarPWM(3000);
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
  esperarEAtualizarPWM(3000);
}

//=========================================================================================================================================

void esperarEAtualizarPWM(unsigned long tempoEspera) {
  unsigned long inicio = millis();
  while (millis() - inicio < tempoEspera) {
    aplicarPWMSeguro(); // Mantem o fade da luz rodando suavemente
    wdt_reset();        // Alimenta o Watchdog para evitar resets
  }
}
