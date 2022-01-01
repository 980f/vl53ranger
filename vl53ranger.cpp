//
// Created by andyh on 12/31/21.
//

#include "nonblocking.h"
#include "vl53ranger.h"

using namespace VL53L0X;

VL53Ranger api;


VL53Ranger::VL53Ranger() : NonBlocking(static_cast<NonBlocking::UserAgent &> (*this)){}

void VL53Ranger::afterProcess(NonBlocking::ProcessRequest process, NonBlocking::ProcessResult stage) {
  switch (process) {
    case OneShot:
      //act on a measurment
      break;
    case Idle:
      break;
    case InitI2c:
      break;
    case InitStatic:
      break;
    case InitData:
      if(stage==Succeeded){
        startProcess(InitStatic);
      }
      break;
    case Continuous:
      break;
    case RefCal:
      break;
    case RateTest:
      break;
    case SetupSpads:
      break;
    case Offset:
      break;
  }
}




/////////////////////////////

void setup(){
  api.setup();
}

void loop(){
  api.loop();
}


#pragma ide diagnostic ignored "EndlessLoop"
int main(){
  setup();
  while(true){
    loop();
  }
}
