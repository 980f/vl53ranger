# vl53ranger
vl53l0x laser ranger, major rework of ST's library and Adafruit's wrapper for it.

Initial import was from zipfile provided by ST micro. Within a few moments I found a bug.

I then got the Adafruit arduino library which among other things compiles the C code as C++. They did not dig through the layers of the original code to get rid of massive redundancy and other space wasting.

I am mutating the code into all C++, and may perhaps write C wrappers to test what issues that may cause. So much of the code is begging to be C++, or suffering because it is not. BTW: C wrappers for C++ code are easy to write. Migrating the official code to C++ would be a good thing for the vendor, presuming that they have C++ tools for all of their processor families.

The first step was to flatten the file hierarchy for ease of management, there may be some Arduino reason for the structure, that can be restored later.

Next was to get rid of hard coded line-wrapping that precludes use of grep to learn the code. When doing 'where/how used' it is highly valuable to have all function parameters on the same line as the invocation. And then there is the simple fact that lots of gratuitous blank lines leads to a lesser amount of program logic on the screen at any moment, reducing the view of the context for each line.

Someone seemed averse to ternaries, the only reason to avoid them is to allow setting of breakpoints depending upon which value is being assigned.

One of ST's worst habits is to only return a status, even when that is a fixed value. If the operation cannot fail then return a value instead of having a pointer parameter. While I will not add exceptions this kind of coding pattern is why they were created, and just like exceptions there are places where the returned status is ignored ;). 

A technique that would help comprehending such blocks of code is to use a do {} while(0); around a series of operations where one will bail out on the first error. One can then do if(! someoperation()) break;  versus setting a variable with each operation and then testing it repeatedly before each successive operation. It looks like a lot of this was to ensure getting to the logging END tracer, something that RAII logging objects tends to quite nicely.

The logging has the occasional failure to have an END for a START. An RAII class will take care of that and reduce the #define gyrations that lack of object oriented thinking caused.

I have moved local variables to places of first use which has turned up many uses of values that were NOT successfully read from the hardware. The dealing with errors is so haphazard, and so unlikely to be recoverable, that most of it is entirely pointless. Logging errors at point of detection for _post-mortem_ analysis seems more appropriate. A counter for each error is not unreasonable. Almost all detectable errors are related to accessing some value from the device, and those are simple comm errors that can affect any value. Anyone caring for robustness will hook up the 'xshut' signal and will cycle it and reinit on failure, or even better yank the power from the chip via some GPIO in order to get a full hardware reset of it. There are sufficiently conductive FET's to make that cheap to do, and many microcontrollers could actually power the device directly from a GPIO, it needs 19ma typical at 2.8V, 40ma peak. 

I am now trying to decide whether to push the Error propagation into a thread_local, available with C11 and C++11, or even perhaps use the legendary setjmp/lomgjmp for most of them.
The core of most errors is I2C communications failure. That can happen in the midst of a set of transactions and if the vl53's firmware is poor transactions might partially execute.
Trying to partially recover seems unlikely, one should reset and restart the vl53 after a loss of I2C comms. If I2C errors are not survivable then we can ignore their possibility and let a watchdog timer or some timeout in the user app deal with the stumbling and fumbling that ignoring the errors may result in.

A microcontoller that is pushed for resources will not have an RTOS and as such will not have threads so a simple global for error analysis seems good enough. Even with multiple threads it is not sane to have more than one thread that actually talks to the device, it only does one thing and that is report distance to target. So, record errors on a global instead of returning them and then returns can be used for values instead of creating explicit locals for reading values when the compiler might be able to keep them on anonymous stack, admittedly only saving 10's or less of stack ram, but making for much more legible code.

Back to logging: the ST setup had 31 channels defined and only two used. Perhaps a third was intended for i2c error logging but the Adafruit module uses Serial.print. Globalizing error reporting will allow dropping of excess layers of calling, saving code and execution time. Object wrapping per 980f's Wire support would greatly simplify the source code retaining easy porting to other platforms, those other platforms being windows/linux/macOS, or ST's HAL stuff (another bucket of worms due to coding in C rather than C++).

So, the next order of attack: object access to I2C values, RAII logging.
 


