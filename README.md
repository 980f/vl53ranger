# vl53ranger
vl53l0x laser ranger, major rework of ST's library and Adafruit's wrapper for it.

Initial import was from zipfile provided by ST micro. Within a few moments I found a bug.

I then got the Adafruit arduino library which among other things compiles the C code as C++. They did not dig through the layers of the original code to get rid of massive redundancy and other space wasting.

I am mutating the code into all C++, and may perhaps write C wrappers to test what issues that may cause. So much of the code is begging to be C++, or suffering because it is not.

The first step was to flatten the file hierarchy for ease of management, there may be some Arduino reason for the structure, that can be restored later.
Next was to get rid of hard coded line-wrapping that precludes use of grep to learn the code. When doing 'where/how used' it is highly valuable to have all function parameters on the same line as the invocation. And then there is the simple fact that lots of gratuitous blank lines leads to a lesser amount of program logic on the screen at any moment, reducing the view of the context for each line.
Someone seemed averse to ternaries, the only reason to avoid them is to allow setting of breakpoints depending upon which value is being assigned.
One of ST's worst habits is to only return a status, even when that is a fixed value. If the operation cannot fail then return a value instead of having a pointer parameter. While I will not add exceptions this kind of coding pattern is why they were created, and just like exceptions there are places where the returned status is ignored ;). 
A technique that would help such blocks of code is to use a do {} while(0); around a series of operations where one will bail out on the first error. One can then do if(! someoperation()) break;  versus setting a variable with each operation and then testing it repeatedly before each successive operation. It looks like a lot of this was to ensure getting to the logging END tracer, something that RAII logging objects tends to quite nicely.

The logging has the occasional failure to have and END for a START. An RAII class will take care of that and reduce the #define gyrations that lack of object oriented thinking caused.
