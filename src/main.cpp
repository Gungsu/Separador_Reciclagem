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
#define Sens_Opt_Alto       35 //35
#define Sens_Opt_Baixo      34 //34
#define Sens_Opt_Passagem   33
#define Sens_Opt_Pass2      32 //Inverte sinal

//Sesores e posição
/*
*  Laranja Difuso Alto       Copo comum
*  HW-201         Baixo      Lata
*  Azul Reflex    Passagem   Copinho pequeno
*  OPtco Par      Pass2      Copo comum
*/

#define ATIVO_S_Alto 1
#define ATIVO 0
#define tempoMinAtivo 4

// put function declarations here:
Ticker tkSec;
uint32_t timer1 = 0;
ESP_FlexyStepper stepper;

typedef struct
{
  uint32_t Opt_Alto_activo;
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

bool comparaTimer(uint32_t tickStart, uint32_t tempoTotal)
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
  if (dif > tempoTotal)
  {
    return true;
  }
  else
  {
    return false;
  }
  return false;
}

void verificandoIndiv(uint16_t PORTA, uint32_t &contador, bool &estado, uint32_t &startTime, bool &timerInit, bool sign = 1) {
  if (digitalRead(PORTA) == sign && !timerInit)
  {
    startTime = timer1;
    timerInit = true;
  } else if (digitalRead(PORTA) != sign) {
    timerInit = false;
    estado = false;
  }
  if (timerInit) {
    if (digitalRead(PORTA) == sign)
    {
      if (comparaTimer(startTime, tempoMinAtivo)) {
        estado = true;
      } else {
        estado = false;
      }
    }
  }
}

Sensores listaSens;

void verificandoestado() {
  //SENSOR OPTO NA POSICAO MAIS ALTA
  verificandoIndiv(Sens_Opt_Alto,listaSens.Opt_Alto_activo,listaSens.Opt_Alto, listaSens.start_time_Opto_Alto, listaSens.timiInit_Opto_Alto);
  // SENSOR OPTO NA POSICAO MAIS BAIXA
  verificandoIndiv(Sens_Opt_Baixo, listaSens.Opt_Baixo_activo, listaSens.Opt_Baixo, listaSens.start_time_Opto_Baixo, listaSens.timiInit_Opto_Baixo);
  // SENSOR OPTO PASSAGEM
  verificandoIndiv(Sens_Opt_Passagem, listaSens.Opt_Pass_activo, listaSens.Opt_Pass, listaSens.start_time_Opto_Pass, listaSens.timiInit_Opto_Pass);
  // Sensor OPTO Passagem2
  verificandoIndiv(Sens_Opt_Pass2, listaSens.Opt_Pass_activo2, listaSens.Opt_Pass2, listaSens.start_time_Opto_Pass2, listaSens.timiInit_Opto_Pass2,0);
  // listaSens.Opt_Alto = digitalRead(Sens_Opt_Alto);
  // listaSens.Opt_Baixo = digitalRead(Sens_Opt_Baixo);
  // listaSens.Opt_Pass = digitalRead(Sens_Opt_Passagem);
  // listaSens.Opt_Pass2 = digitalRead(Sens_Opt_Pass2);
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
  stepper.setSpeedInStepsPerSecond(180);
  stepper.setAccelerationInStepsPerSecondPerSecond(100);
  stepper.setDecelerationInStepsPerSecondPerSecond(100);

  pinMode(Solenoide, OUTPUT);
  pinMode(LedVerde, OUTPUT);
  pinMode(LedAmarelo, OUTPUT);
  pinMode(LedVermelho, OUTPUT);
  pinMode(Driver_Enable, OUTPUT);

  pinMode(Sens_Opt_Alto, INPUT);
  pinMode(Sens_Opt_Baixo, INPUT);
  pinMode(Sens_Opt_Passagem, INPUT);
  pinMode(Sens_Opt_Pass2, INPUT);
}

#define TESTE

void loop() {
  #ifdef TESTE
    verificandoestado();
    Serial.print(listaSens.start_time_Opto_Alto);
    Serial.print(" ");
    Serial.print(listaSens.start_time_Opto_Baixo);
    Serial.print(" ");
    Serial.print(listaSens.start_time_Opto_Pass);
    Serial.print(" ");
    Serial.println(listaSens.start_time_Opto_Pass2);
    delay(10);
    return;
  #endif
  verificandoestado();
  if(listaSens.Opt_Alto && listaSens.Opt_Baixo) {
    //Latinha ACIONAR SOLENOIDE
    controleFarol(1,0,0);
    delay(1000);
    controleFarol(0, 1, 0);
    digitalWrite(Solenoide,1);
  }
  if (listaSens.Opt_Alto && !listaSens.Opt_Baixo) {
    //ACIONAR PWM PELO TEMPO NECESSARIO PARA CHEGAR NO SEGUNDO BURACO
    controleFarol(1, 0, 0);
    //Acionar PWM
    digitalWrite(Driver_Enable, ATIVO);
    delay(500);
    stepper.moveRelativeInSteps(1000);
    delay(2000);
    stepper.moveRelativeInSteps(-1000);
    delay(100);
    digitalWrite(Driver_Enable, !ATIVO);
    controleFarol(0, 1, 0);
  }
  if (listaSens.Opt_Pass) {
    //COPINHO DE CAFÉ PASSOU
    controleFarol(0, 1, 0, 1);
    delay(500);
    controleFarol(1, 0, 0);
  }
  if (listaSens.Opt_Pass2)
  {
    // COPO GRANDE PASSOU
    controleFarol(0, 1, 1, 1);
    delay(500);
    controleFarol(1, 0, 0);
  }
  if(!(listaSens.Opt_Alto && listaSens.Opt_Baixo && listaSens.Opt_Pass)) {
    //NENHUM SENSOR ATIVO
    controleFarol(0, 0, 1);
  }
}