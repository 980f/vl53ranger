//no multiinclude guard.

#define LOG_FUNCTION_START(fmt, ...)   _LOG_FUNCTION_START(TRACE_MODULE_PLATFORM, fmt, ## __VA_ARGS__)
#define LOG_FUNCTION_END(status, ...)    _LOG_FUNCTION_END(TRACE_MODULE_PLATFORM, status, ## __VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)     _LOG_FUNCTION_END_FMT(TRACE_MODULE_PLATFORM, status, fmt, ## __VA_ARGS__)

