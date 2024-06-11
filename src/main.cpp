#include <Arduino.h>
#include <Ticker.h>
#include "ESP_FlexyStepper.h"
//PORTAS
//Atuadores OUT
#define Solenoide     32
#define Driver_Direct 33
#define Driver_Pulse  14
#define LedVerde      25
#define LedAmarelo    26
#define LedVermelho   27

//Sensores IN
#define Sens_Opt_Alto      19
#define Sens_Opt_Baixo     18
#define Sens_Opt_Passagem  5

#define ATIVO 1
#define tempoMinAtivo 10

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

void verificandoIndiv(uint16_t PORTA, uint32_t &contador, bool &estado, uint32_t &startTime, bool &timerInit) {
  if (digitalRead(PORTA) == ATIVO && !timerInit)
  {
    startTime = timer1;
    timerInit = true;
  } else if (digitalRead(PORTA) != ATIVO) {
    timerInit = false;
  }
  if (timerInit) {
    if (digitalRead(PORTA) == ATIVO)
    {
      if (comparaTimer(startTime, tempoMinAtivo)) {
        estado = true;
        timerInit = false;
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
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  tkSec.attach(0.01, everySecond); // conta +1 no timer a cada 10ms do clock

  stepper.connectToPins(Driver_Pulse,Driver_Direct);
  stepper.setSpeedInStepsPerSecond(800);
  stepper.setAccelerationInStepsPerSecondPerSecond(200);
  stepper.setDecelerationInStepsPerSecondPerSecond(200);

  pinMode(Solenoide, OUTPUT);
  pinMode(LedVerde, OUTPUT);
  pinMode(LedAmarelo, OUTPUT);
  pinMode(LedVermelho, OUTPUT);

  pinMode(Sens_Opt_Alto, INPUT);
  pinMode(Sens_Opt_Baixo, INPUT);
  pinMode(Sens_Opt_Passagem, INPUT);
}

void loop() {
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
    stepper.moveRelativeInSteps(1000);
    delay(2000);
    stepper.moveRelativeInSteps(-1000);
    delay(100);
    controleFarol(0, 1, 0);
  }
  if (listaSens.Opt_Pass) {
    //COPINHO DE CAFÉ PASSOU
    controleFarol(1, 0, 0);
    delay(500);
    controleFarol(0, 1, 0, 1);
  }
  if(!(listaSens.Opt_Alto && listaSens.Opt_Baixo && listaSens.Opt_Pass)) {
    //NENHUM SENSOR ATIVO
    controleFarol(0, 0, 1);
  }
}