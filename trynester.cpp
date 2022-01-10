//
// Created by andyh on 12/28/21.
//

#include "trynester.h"

template<> Stacked<Thrower> *Stacked<Thrower>::tos = nullptr;

template<> Stacked<LocationStack> *Stacked<LocationStack>::tos = nullptr;

LocationStack::Logger *LocationStack::logger = nullptr;

bool LocationStack::onEntry =true;//be verbose until we get around to turning it off, such as when initLogging is called.


void LocationStack::logException(int thrown) {
  logger->exception(thrown);
  auto now = logger->stamper();//read clock just once, roughly the time of the exception.
  walk([now](LocationStack *item) {
    logger->reportElapsed(item->element, now - item->timestamp);
    return true;//report full context, not just back to catch point
  });
}

LocationStack::LocationStack(LocationStack::Element &element) : element(element) {
  timestamp=0;//logger needs this for starts to print as absolute
  if(logger){
    auto stamp = logger->stamper();
    if(onEntry){
      logger->reportElapsed(element,stamp);
    }
    timestamp=stamp;
  }

}

void Thrower::operator()(int errorcode) {
  if (stacked) {//using as an 'initialized' boolean
    longjmp(opaque, errorcode);
  } else {
    //unhandled exception handler goes here.
  }
}

/** recursive routines don't play nice in this system. We should move the timestamp to the stacker. */
static void tracer_example(unsigned recurse) {
  static LocationStack::Element tracer(__FUNCTION__, __FILE__, __LINE__);
  LocationStack namedoesntmatter(tracer);
  if (recurse--) {
    tracer_example(recurse);
  } else {
    //check:  should see records identical except for timestamp
    LocationStack::walk([](Stacked<LocationStack> *item) {
      //item.element.timestamp = ~0;//testing access
      return true;
    });
  }
}

static int throw_example() {
  TRACE_ENTRY  //without one of these you don't get a proper idea of where the problem occured.
  TRY {
      Throw(42); //throws to self but use the following
      THROW(89);  //also throws to self
      return ~1;
    CATCH(42)
      return 42;//something to test compilation
    CATCH(-42)
      THROW(99); //throws where it should
    UNCAUGHT
      Throw(-42);//this is throw to self
      return ~0;
  }
  return 0;
}

#include <Arduino.h>
class DummyLogger : public LocationStack::Logger{
public:
  LocationStack::Ticks stamper() override {
    return micros();
  }

  void reportElapsed(const LocationStack::Element &element, LocationStack::Ticks elapsed) override {
    Serial.print(element.function);
    Serial.print(" took ");
    Serial.print(elapsed);
    Serial.print(" (");
    Serial.print(element.file);
    Serial.print(element.line);
    Serial.print(") ");
  }

  void exception(int throwncode) override {
    Serial.print(throwncode);
    Serial.println(F("Stack trace:"));
  }
};
