/** Example of application for using VST VL53L0x device
 * Copyright 2022 by Andrew Heilveil (github/980f)
*/

#include "nonblocking.h" // the driver
#include "vl53ranger.h"  // this basic demo application


//#ifndef ARDUINO  //then dummy up the interface so that we can compile
unsigned  digitalRead(unsigned ){ return 0;}
//#endif

using namespace VL53L0X;

VL53Ranger api;

VL53Ranger::VL53Ranger() noexcept : NonBlocking(static_cast<NonBlocking::UserAgent &> (*this)){}

void VL53Ranger::processEvent(NonBlocking::ProcessRequest process, NonBlocking::ProcessResult stage) {
  switch (process) {
    case OneShot:
      //?act on a measurement, or has that already been done?
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
    case RateTest:
      break;
    case SetupSpads:
      break;
    case Offset:
      break;
    case CalVhvPhase:
      break;
    case CalPhase:
      break;
    case CrossTalk:
      break;
  }
}

bool VL53Ranger::gpioSignal() {
  return digitalRead(arg.gpioPin)==0;//hard coding low active until we have configuration for it through the layers
}


/////////////////////////////

void setup(){
  api.agent.arg.sampleRate_ms=33;// a leisurely rate. Since it is nonzero continuous measurement will be initiated when the device is capable of it
  api.agent.arg.gpioPin=3;
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
