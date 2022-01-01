#pragma  once
//
// Created by andyh on 12/31/21.
//


#include "nonblocking.h"

class VL53Ranger : public VL53L0X::NonBlocking, NonBlocking::UserAgent{
public:
  VL53L0X_Application(): NonBlocking(static_cast<UserAgent &> (*this)){}

  void afterProcess(ProcessRequest process, ProcessResult stage) override {
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
};
