/** Example of application for using VST VL53L0x device
 * Copyright 2022 by Andrew Heilveil (github/980f)
*/

#include "nonblocking.h" // the driver
#include "vl53ranger.h"  // this basic demo application
#include "vl53l0x_platform_log.h"

//#ifndef ARDUINO  //then dummy up the interface so that we can compile
unsigned digitalRead(unsigned) {
  return 0;
}


//#endif

using namespace VL53L0X;

VL53Ranger api;

VL53Ranger::VL53Ranger() noexcept: NonBlocking(static_cast<NonBlocking::UserAgent &> (*this)) {
}

void VL53Ranger::processEvent(NonBlocking::ProcessRequest process, NonBlocking::ProcessResult stage) {
  switch (process) {
    case Operate: //receives measurements
      switch (stage) {
        case Busy:
          break;
        case Succeeded: {
          auto distance = agent.arg.theRangingMeasurementData.Range.distance();
        }
          break;
        case Failed:
          break;
      }
      break;
    case Idle: //acknowledges pwower down et al.
      break;

    case InitStatic://unnecessary or start operating
      break;
    case InitData:
      if (stage == Failed) {
        //serious system issue. Likely no I2C connection.
      }
      break;
    case RateTest://here's your diagnostic data
      break;
    case SetupSpads://only static init should care
      break;
    case Offset: //you may wish to backup parameters
      break;
    case CalVhvPhase://you may wish to backup parameters
      break;
    case CalPhase://you may wish to backup parameters
      break;
    case CrossTalk://you may wish to backup parameters
      break;
  }
}

/** */
bool VL53Ranger::gpioSignal() {
  return digitalRead(arg.gpioPin) == 0;//hard coding low active until we have configuration for it through the layers
}

/////////////////////////////


void setup() {
  api.agent.arg.sampleRate_ms = 33;// a leisurely rate. Since it is nonzero continuous measurement will be initiated when the device is capable of it
  api.agent.arg.gpioPin = 3;//todo: #define near top of file/class
  VL53L0X::initLogging(); //api doesn't know how logging is configured
  api.setup(NonBlocking::DataStream);//stream data.
}

void loop() {
  //check UI for continuous on/off/powerdown
  api.loop();
}

///
//pretend to be an Arduino for testing compilation.
#pragma ide diagnostic ignored "EndlessLoop"

int main() {
  setup();
  while (true) {
    loop();
  }
}
