//
// Created by andyh on 12/31/21.
//

#include "nonblocking.h"
#include "vl53ranger1.h"

using namespace VL53L0X;

VL53L0X_Application api;


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
