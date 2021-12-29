
//formerly coerced in api.h rather than being a platform option.
#define USE_I2C_2V8 1


//define to something other than 0 if calibration routines will be needed, else they will be carefully excised from the build to minimize code footprint 
#define IncludeCalibrators 1

//define to something other than 0 if histogram modality is to be present in the API
#define IncludeHistogramming 0

#define HaveRoiZones 0

//set to zero for compile time notification of a function that is not implemented
#define IncludeNotimplemented 1

#ifdef UseEmptyStrings
#error "the empty strings option is now implemented by setting a file link from vl53l0x.empty.text to vl53l0x.text"
#endif

//will be trying to remove all copying and just return pointers.
#define COPYSTRING(target, string) target=string
#define VL53L0X_COPYSTRING(target, string) COPYSTRING(target, string)
