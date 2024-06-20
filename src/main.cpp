#include <Arduino.h>
#include <Ticker.h>
#include "ESP_FlexyStepper.h"
//PORTAS
//Atuadores OUT
#define Solenoide     23
#define Driver_Direct 4
#define Driver_Pulse  2
#define Driver_Enable 18
#define LedVerde      22
#define LedAmarelo    21
#define LedVermelho   19

//Sensores IN
#define Sens_Opt_Alto       35 //35  26
#define INV_O_ALTO          0  //0  1
#define Sens_Opt_Baixo      34 //34  25
#define INV_O_BAIXO         1  //1  1
#define Sens_Opt_Passagem   33 //33
#define INV_O_PASS          0  //0  1
#define Sens_Opt_Pass2      32 //Inverte sinal
#define INV_O_PASS2         1  //1  1
#define SETINPUT            INPUT
#define AT_SOLEN            1  //Ativar solenoide com sinal: ___
//Sesores e posição
/*
*  Laranja Difuso Alto       Copo comum
*  HW-201         Baixo      Lata
*  Azul Reflex    Passagem   Copinho pequeno
*  OPtco Par      Pass2      Copo comum
*/

#define ATIVO_S_Alto 1
#define ATIVO 0
#define tempoMinAtivo 100

// put function declarations here:
Ticker tkSec;
uint32_t timer1 = 0;
ESP_FlexyStepper stepper;
bool waitCopoGrande = false;

typedef struct
{
  uint32_t offtime_Opt_Alto;
  uint32_t start_time_Opto_Alto;
  bool Opt_Alto;
  bool timiInit_Opto_Alto;

  uint32_t Opt_Baixo_activo;
  uint32_t start_time_Opto_Baixo;
  bool Opt_Baixo;
  bool timiInit_Opto_Baixo;

  uint32_t Opt_Pass_activo;
  uint32_t start_time_Opto_Pass;
  bool Opt_Pass;
  bool timiInit_Opto_Pass;

  uint32_t Opt_Pass_activo2;
  uint32_t start_time_Opto_Pass2;
  bool Opt_Pass2;
  bool timiInit_Opto_Pass2;
} Sensores;

bool comparaTimer(uint32_t tickStart, uint32_t tempoTotal, bool x=0)
{
  uint64_t dif;
  if (tickStart > timer1)
  {
    dif = timer1 + tickStart;
  }
  else
  {
    dif = timer1 - tickStart;
  }
  if(!x) {
    if (dif > tempoTotal)
    {
      return true;
    }
    else
    {
      return false;
    }
  } else {
    if (dif > uint32_t(tempoTotal/2))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

void verificandoIndiv(uint16_t PORTA, uint32_t &offTime, bool &estado, uint32_t &startTime, bool &timerInit, bool sign = 1)
{
  if (digitalRead(PORTA) == sign && !timerInit)
  {
    startTime = timer1;
    timerInit = true;
    offTime = 0;
  } else if (digitalRead(PORTA) != sign && timerInit) {
    if(offTime == 0) {
      offTime = timer1;
    }
    if(comparaTimer(offTime, tempoMinAtivo,1)){
      timerInit = false;
      estado = false;
      offTime = 0;
    }
  }
  if (timerInit) {
    if (digitalRead(PORTA) == sign)
    {
      offTime = 0;
      if (comparaTimer(startTime, tempoMinAtivo)) {
        estado = true;
      }
    }
  }
}

Sensores listaSens;

void verificandoestado() {
  //SENSOR OPTO NA POSICAO MAIS ALTA
  // SENSOR OPTO NA POSICAO MAIS BAIXA
  //verificandoIndiv(Sens_Opt_Baixo, listaSens.Opt_Baixo_activo, listaSens.Opt_Baixo, listaSens.start_time_Opto_Baixo, listaSens.timiInit_Opto_Baixo);
  // SENSOR OPTO PASSAGEM
  //verificandoIndiv(Sens_Opt_Passagem, listaSens.Opt_Pass_activo, listaSens.Opt_Pass, listaSens.start_time_Opto_Pass, listaSens.timiInit_Opto_Pass);
  // Sensor OPTO Passagem2
  //verificandoIndiv(Sens_Opt_Pass2, listaSens.Opt_Pass_activo2, listaSens.Opt_Pass2, listaSens.start_time_Opto_Pass2, listaSens.timiInit_Opto_Pass2,0);
  verificandoIndiv(Sens_Opt_Alto,listaSens.offtime_Opt_Alto,listaSens.Opt_Alto,listaSens.start_time_Opto_Alto,listaSens.timiInit_Opto_Alto);
  //listaSens.Opt_Alto = (INV_O_ALTO) ? !digitalRead(Sens_Opt_Alto) : digitalRead(Sens_Opt_Alto);
  listaSens.Opt_Baixo = (INV_O_BAIXO) ? !digitalRead(Sens_Opt_Baixo) : digitalRead(Sens_Opt_Baixo);
  listaSens.Opt_Pass = (INV_O_PASS) ? !digitalRead(Sens_Opt_Passagem) : digitalRead(Sens_Opt_Passagem);
  listaSens.Opt_Pass2 = (INV_O_PASS2) ? !digitalRead(Sens_Opt_Pass2) : digitalRead(Sens_Opt_Pass2);
  // Serial.print(listaSens.Opt_Alto);
  // Serial.print(" ");
  // Serial.print(listaSens.Opt_Baixo);
  // Serial.print(" ");
  // Serial.print(listaSens.Opt_Pass);
  // Serial.print(" ");
  // Serial.println(listaSens.Opt_Pass2);
}

void everySecond()
{
  timer1++;
}

void controleFarol(bool vermelho, bool amarelo, bool verde, uint8_t modo = 0) {
  if (modo == 0) {
    digitalWrite(LedVerde, verde);
    digitalWrite(LedAmarelo, amarelo);
    digitalWrite(LedVermelho, vermelho);
  }
  if (modo == 1) { //Piscar 3 vezes com delay de 500ms
    uint8_t x = 3;
    while(x--) {
      digitalWrite(LedVerde, verde);
      digitalWrite(LedAmarelo, amarelo);
      digitalWrite(LedVermelho, vermelho);
      delay(500);
      digitalWrite(LedVerde, !verde);
      digitalWrite(LedAmarelo, !amarelo);
      digitalWrite(LedVermelho, !vermelho);
      delay(500);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  tkSec.attach(0.01, everySecond); // conta +0,01 no timer a cada 10ms do clock

  stepper.connectToPins(Driver_Pulse,Driver_Direct);
  stepper.setSpeedInStepsPerSecond(18000);
  stepper.setAccelerationInStepsPerSecondPerSecond(100);
  stepper.setDecelerationInStepsPerSecondPerSecond(100);

  pinMode(Solenoide, OUTPUT);
  pinMode(LedVerde, OUTPUT);
  pinMode(LedAmarelo, OUTPUT);
  pinMode(LedVermelho, OUTPUT);
  pinMode(Driver_Enable, OUTPUT);

  pinMode(Sens_Opt_Alto, SETINPUT);
  pinMode(Sens_Opt_Baixo, SETINPUT);
  pinMode(Sens_Opt_Passagem, SETINPUT);
  pinMode(Sens_Opt_Pass2, SETINPUT);

  digitalWrite(Solenoide, !AT_SOLEN);
  digitalWrite(Driver_Enable, ATIVO);
}

#define DEBUG_S_MOTOR

void loop() {
  #ifdef CALIB
    digitalWrite(Driver_Enable, ATIVO);
    while(true) {
      verificandoestado();
      if(listaSens.Opt_Alto) {  
        stepper.moveRelativeInSteps(-12000);
        Serial.println(1);
      }
      if(listaSens.Opt_Baixo) {
        stepper.moveRelativeInSteps(12000);
        Serial.println(0);
      }
      if(listaSens.Opt_Pass) {
        digitalWrite(Solenoide, AT_SOLEN);
        delay(500);
        digitalWrite(Solenoide, !AT_SOLEN);
        delay(100);
      }
    }
    return;
  #endif
  #ifdef TESTSENS
    verificandoestado();
    Serial.print(listaSens.Opt_Alto);
    Serial.print(" ");
    Serial.print(listaSens.Opt_Baixo);
    Serial.print(" ");
    Serial.print(listaSens.Opt_Pass);
    Serial.print(" ");
    Serial.println(listaSens.Opt_Pass2);
    return;
  #endif
  verificandoestado();
  if(listaSens.Opt_Alto && listaSens.Opt_Baixo) {
    //Latinha ACIONAR SOLENOIDE
    Serial.println("Latinha ACIONAR SOLENOIDE");
    controleFarol(1,0,0);
    delay(1000);
    controleFarol(0, 1, 0);
    digitalWrite(Solenoide, AT_SOLEN);
    delay(1000);
    digitalWrite(Solenoide, !AT_SOLEN);
    listaSens.Opt_Alto = 0;
  }
  if (listaSens.Opt_Alto && !listaSens.Opt_Baixo) {
    //ACIONAR PWM PELO TEMPO NECESSARIO PARA CHEGAR NO SEGUNDO BURACO
    Serial.println("ACIONAR PWM PELO TEMPO NECESSARIO");
    waitCopoGrande = true;
    controleFarol(1, 0, 0);
    //Acionar PWM
    #ifndef DEBUG_S_MOTOR
        stepper.moveRelativeInSteps(-12000);
    #endif
  }
  if (listaSens.Opt_Pass) {
    //COPINHO DE CAFÉ PASSOU
    Serial.println("COPINHO DE CAFE PASSOU");
    controleFarol(0, 1, 0, 1);
    delay(500);
    controleFarol(1, 0, 0);
  }
  if (waitCopoGrande) {
    while (!listaSens.Opt_Pass2)
    {
      // COPO GRANDE PASSOU
      Serial.println("ESPERANDO COPO GRANDE PASSAR");
      verificandoestado();
    }
    Serial.println("COPO GRANDE PASSOU");
    waitCopoGrande = false;
    #ifndef DEBUG_S_MOTOR
      stepper.moveRelativeInSteps(12000);
    #endif
    controleFarol(0, 1, 1, 1);
    delay(500);
    controleFarol(1, 0, 0);
    delay(500);
    controleFarol(0, 1, 0);
  }
  if(!(listaSens.Opt_Alto && listaSens.Opt_Baixo && listaSens.Opt_Pass)) {
    //NENHUM SENSOR ATIVO
    Serial.println("AGUARDANDO PROXIMO ITEN");
    controleFarol(0, 0, 1);
  }
}