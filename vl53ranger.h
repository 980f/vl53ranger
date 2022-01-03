#pragma  once
//
// Created by andyh on 12/31/21.
//


#include "nonblocking.h"

class VL53Ranger : public VL53L0X::NonBlocking, VL53L0X::NonBlocking::UserAgent{
public:
  VL53Ranger() noexcept;

  void afterProcess(ProcessRequest process, ProcessResult stage) override;
};
