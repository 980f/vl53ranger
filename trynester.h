// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/28/21.
/** These classes wrap setjmp/longjmp usage for microcontrollers for which c++'s exception handling is too expensive.
 * It is not sane to use these with interrupts or multiple threads, although a threadlocal instance might work as expected.
 *
 * Unlike true exception handling you do NOT get destruction of items on the stack. That means that if you have any dynamic allocation going on you should not use this.
 * One could complexify the class by registering destructors with it, but then you should probably not be disabling exceptions, this module is intended for resource limited microcontrollers.
 *
 * Each Thrower is independent of others, if you have multiple ones it will be confusing to deal with that.
 * Each caller of a function which might throw has to know all of the Throwers in the call tree.
 *
 *
 * The TryNester allows you to trap e
 * */

#ifndef TRYNESTER_H
#define TRYNESTER_H

#include <csetjmp>

/** wrap a class around jmp_buf for exception handling via longjmp.
 * There is one of these for each exception source, which is not all that convenient if you have more than one.
 * */
class Thrower {
  jmp_buf opaque; //an array of bytes.
  /** a guard in case we try to throw before there has ever been a Catch() */
  bool enabled = false;
public:
  /** throw!
   * if there is no receiver then we return the passed argument so that you can:
   * return Thrower(exception, returnIfExceptionsDisbled);
   * */
  template<typename Altreturn> Altreturn
  operator()(int errorcode, Altreturn alt) {
    if (enabled) {
      longjmp(opaque, errorcode);
    }
    return alt;
  }

  void operator()(int errorcode) {
    if (enabled) {
      longjmp(opaque, errorcode);
    }
  }

  operator jmp_buf() {
    return opaque;
  }

  //we can't put setjmp in a method as the return would be to that method call.
  //but that actually does work ....
  int Catch() {
    enabled=true;
    return setjmp(opaque);
  }

  ~Thrower(){
    enabled=false;
  }
};

/** creating one of these preserves then replaces the jmp_buf values, so on destruction the original jmp_buf stuff is restored.
 *
 * when the throw/longjmp occurs the constructor is resumed so we capture the lomgjmp info on this object:
 *
 * {
 *    variables that are used by both the try and catch blocks must be declared prior to the TryNester
 *   TryNester catcher(global jmp_buf);
 *   if(!catcher){
 *      //try stuff
 *   } else {
 *     switch(catcher.error){
 *     //handle each error
 *     }
 *   }
 * }// original trap restored here.
 *
 * If you have a late enough version of C++ you can do this:
 *
 *  if(TryNester catcher(global jmp_buf)){
 *
 *  } else {
 *    switch(catcher.errorcode){
 *    }
 *  }
 *
 * */
class TryNester {
  /** the try point */
  jmp_buf opaque;
  bool enabled;
  /** for passing the longjmp returned value out of the constructor which contains the setjmp */
  int errorcode;
  /** redirect the thrower */
  jmp_buf &pushed;

  void pop(){
    enabled=false;
    memcpy(&pushed, &opaque, sizeof(jmp_buf));
  }
public:
  TryNester(jmp_buf &pushed) : pushed(pushed) {
//    opaque=pushed:
    memcpy(&opaque, &pushed, sizeof(jmp_buf));
    enabled=true;
    errorcode = setjmp(pushed); // NOLINT(cert-err52-cpp)
  }

  /** */
  operator bool() const {
    return errorcode == 0;
  }

  void rethrow(){
    //todo: restore jmpbuf (for timing reasons) and longjmp
    pop();
    longjmp(opaque,errorcode);
  }

  ~TryNester() {
    pop();
  }
};

#endif //VL53_TRYNESTER_H
