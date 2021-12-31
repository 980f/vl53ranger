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
 * The TryNester allows you to trap exceptions at differ call depths.
 *
 * The LocationStack is an addon for getting better info on the cause of an exception.
 * It is tempting to force one on the user, but we will use a pointer to make that optional.
 * */

#ifndef TRYNESTER_H
#define TRYNESTER_H

#include <csetjmp>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err52-cpp"   //this file exists to manage lomgjmp when exceptions are too expensive to use.

/** simple stack via linked list, intended to preserve block nesting information when a psuedo exception is thrown.
 * All objects to push must be allocated locally, not statically.
 * The tos is a statically allocated pointer and should be used for all operations. For coding simplicity */
template<typename Element> class Stacked {
protected:
  Stacked *stacked = nullptr;

public:
  /** all throws should be called on this object:
 * if using threads this must be a thread_local
 * */
  static Stacked *tos;

  /** this will only work for certain classes and certain compilers */
  static Element *top() {
    return reinterpret_cast<Element *>(tos);
  }

  /**
   * tos is a pointer, you can call tos->notEmpty even if tos is a nullptr */
  bool notEmpty() const {
    return this != nullptr;
  }

  /** the visitor is given a pointer to what is a base class of the class being stacked.
   * return true if walk is to continue, false to stop it (useful for pealing off part of the top such as only the stack from the try to the throw) */
  using Visitor = bool (*)(Stacked &);

  static void walk(Visitor visitor) {
    for (Stacked *item = tos; item && visitor(*item); item = item->stacked);
  }

  using Invisitor = bool (*)(Element &);
  /** the visitor is given a pointer to what is presumed to be the derived being stacked, but that only works for some such classes.
   * return true if walk is to continue, false to stop it */
  static void walk(Invisitor visitor) {
    for (Stacked *item = tos; item && visitor(*reinterpret_cast<Element*>(item)); item = item->stacked);
  }

protected:
  /** pop is used when chopping the stack when an exception is thrown, as well as when the stack frame is ended via normal execution.
   * it is idempotent= no matter how many times it is called on an object the stack just pops once. */
  void pop() const {
    tos = stacked;
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
  /** a STATIC one of these may be created for each function, so that a backtrace can be printed after an exception has been caught without using stale objects.
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
    bool reportOnExit = false;

    //constructor for static use case:
    Element(const char *function, const char *file, unsigned line) : function(function), file(file), line(line) {
    }
  };

private:
  const Element &element;

  using Ticks = unsigned long; //declspec your timestamp source.
  Ticks timestamp = 0;

public:// until we have setters your application does assignments to these function pointers:

  using TickSource = Ticks(*)();
  static TickSource stamper;

  using Reporter = void (*)(const Element &, Ticks /*elapsed*/, bool /*onException*/);
  static Reporter reportElapsed;

private:
  void pop(bool onException = false) {
    if (element.reportOnExit) {
      if (reportElapsed) {
        reportElapsed(element, stamper() -timestamp, onException);
      }
    }
    Stacked::pop();
  }

public:
  LocationStack(Element &element) : element(element) {
    timestamp = stamper ? stamper() : 0;//todo:2 might make this conditional upon reportOnExit, but the timestamp could be handy when debugging
  }

  ~LocationStack() {
    pop(false);
  }

  /** recursive routines don't play nice in this system. We should move the timestamp to the stacker. */
  static void example(unsigned recurse) {
    static LocationStack::Element tracer(__FUNCTION__, __FILE__, __LINE__);
    LocationStack namedoesntmatter(tracer);
    if (recurse--) {
      example(recurse);
    } else {
      //check:  should see records identical except for timestamp
      walk([](LocationStack &item) {
        //item.element.timestamp = ~0;//testing access
        return true;
      });
    }
  }
};

#define TRACE_ENTRY    static LocationStack::Element tracer(__FUNCTION__, __FILE__, __LINE__);LocationStack namedoesntmatter(tracer);


/** wrap a class around jmp_buf for exception handling via longjmp.
 * There is one of these for each exception source, which is not all that convenient if you have more than one.
 *
 * A statically created one is the top of stack, and is recognized from an actual try by its null tryClause
 * */
class Thrower : public Stacked<Thrower> {
  jmp_buf opaque; //an array of bytes that magically gets us back to where setjmp was called.

  /** for passing the longjmp provided value out of the constructor which contains the setjmp */
  int thrown = 0;
//
//  //we wish to stack contexts so each try block creates a thrower which links into a list operating as a stack, and throwers of exceptions use the TOS instance.
//  Thrower *stacked = nullptr;
//
//  void pop() {
//    tos = stacked;
//  }

public:
//  /** all throws should be called on this object:
//   * if using threads this must be a thread_local
//   * */
//  static Thrower *tos = nullptr;

  /** construction pushes on the exception context stack and runs the setjmp.
  Thrown exceptions appear at the object construction point in the code, but after the execution of allocation of the object.
   */
  Thrower() {
//    this->stacked = tos;
//    tos = this;
    thrown = setjmp(opaque);
    if (thrown) {//then we got here due to a throw on the tos instance, or rarely a "throw to self"
      //we have a design decision to make here, where should exceptions in the exception handlers get handled?
      //if we do what C++ does (a reasonable choice by the principle of least surprise) then:
      pop();//can't wait until context exits to point exceptions outside of this instance
      //if we have a peculiar need to join some other catch clause of the present handler we can throw via its local thrower.
    }
  }

  /** invoked on a named constructor instance can be used to make the if() {} block  the try block and the else{} the catches */
  operator bool() const {
    return thrown == 0;
  }

  /** read only to public, operator int() not used as compiler gets it confused with operator bool */
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
  void operator()(int errorcode) {
    if (stacked) {//using as an 'initialized' boolean
      longjmp(opaque, errorcode);
    } else {
      //unhandled exception handler goes here.
    }
  }

  /** leaving try/catch */
  ~Thrower() {
    pop();
  }

#define Throw  (*reinterpret_cast<Thrower *>(Thrower::tos))

  static void example() {
    TRACE_ENTRY
    if (Thrower tryblock; tryblock) {  ///could macro Try   from the if to the curly
      Throw(42);
    } else            //and macro Catch  else switch (tryblock.throne())
      switch (tryblock.thrown) {
        case 42:
          tryblock.thrown+=2;//something to test compilation
          break;
        default:
          tryblock(tryblock.thrown);//this is a rethrow
          break;
      }
  }

  static void example2() {
    TRACE_ENTRY

    switch (Thrower tryblock;tryblock.thrown) {
      case 0:
        Throw(42);
        break;
      case 42:
        tryblock.thrown+=2;//something to test compilation
        break;
      default:
        tryblock(tryblock.thrown);//this is a rethrow
        break;
    }
  }
};

static int example3() {
  TRACE_ENTRY
  Thrower tryblock;
  switch (Thrower tryblock; int(tryblock)) {
    case 0:
      Throw(42);
      break;
    case 42:
      return 42;//something to test compilation
    default:
      tryblock(tryblock);//this is a rethrow
      break;
  }
}


#pragma clang diagnostic pop
#endif //VL53_TRYNESTER_H
