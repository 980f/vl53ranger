#pragma  once
//
// Created by andyh on 12/31/21.
//


#include "nonblocking.h"

class VL53Ranger : public VL53L0X::NonBlocking, VL53L0X::NonBlocking::UserAgent{
public:
  VL53Ranger() noexcept;

  void processEvent(ProcessRequest process, ProcessResult stage) override;
  /** called when we are waiting for a measurement and have been told to configure the VL53 signal and so on.*/
   bool gpioSignal() override;

};
