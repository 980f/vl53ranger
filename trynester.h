// Copyright 2021 by Andy Heilveil (github/980f)
// Created by andyh on 12/28/21.
/**
 * These classes wrap setjmp/longjmp usage for microcontrollers for which c++'s exception handling is too expensive.
 * It is not sane to use these with interrupts or multiple threads, although a threadlocal instance might work as expected.
 *
 * Unlike true exception handling you do NOT get destruction of items on the stack. That means that if you have any dynamic allocation going on you should not use this.
 * One could complexify the class by registering destructors with it, but then you should probably not be disabling exceptions, this module is intended for resource limited microcontrollers.
 *
 * The LocationStack is an attempt at getting better info on the cause of an exception.
 * */

#ifndef TRYNESTER_H
#define TRYNESTER_H

#include <csetjmp>

#pragma ide diagnostic ignored "cert-err52-cpp"   //this file exists to manage lomgjmp when exceptions are too expensive to use.

/** simple stack via linked list, intended to preserve block nesting information when a psuedo exception is thrown.
 * All objects to push must be allocated locally, not statically.
 * The tos is a statically allocated pointer and should be used for all operations. For coding simplicity */
template<typename Element> class Stacked {
protected:
  Stacked *stacked = nullptr;
//todo
//  /** cache depth value for printing convenience */
//  unsigned depth;
//
//  static unsigned Depth;
public:
  /** all throws should be called on this object:
 * if using threads this must be a thread_local
 * */
  static Stacked *tos;

  static Element *top() {
    return tos ? tos->guts() : nullptr;
  }

  /**
   * tos is a pointer, you can call tos->notEmpty even if tos is a nullptr */
  bool notEmpty() const {
    return this != nullptr;
  }

  /**
  /** this will only work for certain classes and certain compilers:
   * the thing stacked, must be declared class MyStackedThing:public Stacked<MyStackedThing>[, other bases] {..}  any other base class MUST follow the Stacked<> one for this to work.
   * This is necessary due to insisting that we operate with RTTI disabled, which is common if you are disabling exeptions which disablement is the reason this module exists.
   * */
  Element *guts() {
    return reinterpret_cast<Element *>(this);
  }

  /** the visitor is given a pointer to what is a base class of the class being stacked.
   * Visitor returns true if walk is to continue, false to stop it (useful for pealing off part of the top such as only the stack from the try to the throw) */
  template<typename Visitor>
  static void walk(Visitor &&visitor) {
    for (Stacked *item = Stacked<Element>::tos; item && visitor(item->guts()); item = item->stacked) {
    }
  }

  /** @returns what to feed to unwind to restore stack to present state */
  static Stacked *mark() {
    return tos;
  }

  /** remove stack elements without notifying anyone. Stops the unwind when it finds @param mark, leaving mark as top of stack.
   * if mark is not in stack then tos is unchanged. */
  static void unwind(Stacked<Element> *mark) {
    if (mark == nullptr) {
      tos = nullptr;
      return;
    }

    walk([mark](Stacked *stackentry) {
      if (stackentry == mark) {//equal addresses
        tos = mark;
        return false;
      }
      return true;
    });
  }

protected:
  /** pop is used when the stack frame is ended via normal execution.
   * it is idempotent= no matter how many times it is called on an object the stack just pops once. */
  void pop() const {
    if (stacked != nullptr) {
      tos = stacked;
    }
    //else it is the root and we shouldn't actually get here.
  }

  Stacked() {
    this->stacked = tos;
    tos = this;
  }

  ~Stacked() {
    pop();
  }
};

/**
 * Each Throw source should have a location stack, to provide a full backtrace on exception.
 * Until we find a use for more than one we make our life easy by having a static instance.
 * */
class LocationStack : public Stacked<LocationStack> {
  /** a STATIC one of these Elements may be created for each function, so that a backtrace can be printed after an exception has been caught without using stale objects.
    * static construction also allows you to have a per-instance flag to control reporting on function exit and perhaps entrance that you can get to with a debugger.
   * */
public:
  class Element {
    friend class LocationStack;

  public:
    const char *const function;
    const char *const file;
    const unsigned line;
    //mutable for debugger based fiddling:
    mutable bool reportOnExit;

    //constructor for static use case:
    Element(const char *function, const char *file, unsigned line,bool reportOnExit = false) : function(function), file(file), line(line) ,reportOnExit(reportOnExit){
    }
  };

  using Ticks = unsigned long; //todo: take in the class for Ticks via a #define
  struct Logger {
    virtual Ticks stamper() = 0;
    /** this gets called as each routine exits */
    virtual void reportElapsed(const Element &, Ticks elapsed) = 0;
    /** this gets called just before reportElapsed gets called on all stack members */
    virtual void exception(int throwncode) = 0;
  };

  static Logger *logger;

  /** designed to only be called on TOS, but can make sense using a local one since that is usually the top of the stack:
   * @return @param throwcode !=0 as a convenience for a migration from another system's error logging call */
  static bool logTrace(int throwcode, const char *function, const char *file, unsigned line) {
    if (logger) {
      //todo:m a list of "errors worth tracing" could be checked here, IE filter out trivial/convenience throws.
      LocationStack::Element tracer(function, file, line);
      tracer.reportOnExit = true;//always trace this guy
      LocationStack convenientrick(tracer);//puts this blocks tracer on top of stack
      logException(throwcode);
    }
    return throwcode != 0;
  }

private:
  const Element &element;
  Ticks timestamp = 0;//timestamp is on object that manages the push and pop so that Element with location information can be const static.

private:
  void pop() {
    if (element.reportOnExit) {
      if (logger) {
        logger->reportElapsed(element, logger->stamper() - timestamp);
      }
    }
    Stacked::pop();
  }

public:
  LocationStack(Element &element) : element(element) {
    timestamp = logger ? logger->stamper() : 0;//todo:2 might make this conditional upon reportOnExit, but the timestamp could be handy when debugging
  }

  ~LocationStack() {
    pop();
  }

  static void logException(int thrown);
};

//you can put a TRACE_ENTRY inside any block statement.
#define TRACE_ENTRY    static LocationStack::Element tracer(__FUNCTION__, __FILE__, __LINE__);LocationStack namedoesntmatter(tracer);

/** wrap a class around jmp_buf for exception handling via longjmp.
 * */
class Thrower : public Stacked<Thrower> {  //class Stacked provides a static member so only one Thrower stack is allowed
  jmp_buf opaque; //an array of bytes that magically gets us back to where setjmp was called.

  /** for passing the longjmp provided value out of the constructor which contains the setjmp */
  int thrown = 0;

  /** thing used by locationstack to synch with try block */
  Stacked<LocationStack> *mark;
public:
  /** this is the global accessor for throwing a psuedo exception.*/
  static int Throw(int error) {
    (*top())(error);//this longjmp's (if stack has been initialized properly and there are no other bugs in this module)
    return error;//appeases compiler, and perhaps we might not actually throw when there is no try block active
  }

  /** construction pushes this object onto the exception context stack and runs the setjmp.
  Thrown exceptions appear at the object construction point in the code, but after the execution of allocation of the object.
   */
  Thrower() { // NOLINT(cppcoreguidelines-pro-type-member-init)
    mark = LocationStack::mark();
    //# base constructor places this on the top of the throw stack before the setjmp is called
    thrown = setjmp(opaque);
    if (thrown) {//then we got here due to a throw on the tos instance, or rarely a "throw to self"
      LocationStack::logException(thrown);
      //we have a design decision to make here, where should exceptions in the exception handlers get handled?
      //if we do what C++ does (a reasonable choice by the principle of least surprise) then:
      LocationStack::unwind(mark);//not part of pop as its own destructor takes care of popping it under normal circumstances.
      pop();//can't wait until context exits to point exceptions outside of this instance
      //if we have a peculiar need to join some other catch clause of the present handler we can throw via its local thrower.
    }
  }

  /** invoked on a named constructor instance can be used to make the if() {} block  the try block and the else{} the catches */
  operator bool() const {
    return thrown == 0;
  }

  /** read only to public */
  operator int() const {
    return thrown;
  }

  /** throw!  This is only called on the tos entity, although one could throw to one's self locally in a try block.
   * if there is no receiver (or particular error code is deemed not worthy of throwing) then we return the passed argument so that you can:
   * return Thrower(exception, returnIfExceptionsDisbled);
   * or value=Thrower(exception, inrangevalue)  //throw or coerce runtime choice
   * */
  template<typename Altreturn> Altreturn
  operator()(int errorcode, Altreturn alt) {
    operator()(errorcode);//which might longjmp instead of returning
    return alt;
  }

  /** throw! (or ignore) */
  void operator()(int errorcode);

  /** leaving try/catch */
  ~Thrower() {
    pop();
  }
};

//see example in matching cpp file for limitations on using the following macros

#define TRY   switch (Thrower Throw; int(Throw)) { case 0:

#define CATCH(code) } break; case code: {

#define UNCAUGHT  } break; default:

// in case we change our mind (again ;) :
#define THROW(error) Thrower::Throw(error)

#endif //VL53_TRYNESTER_H
