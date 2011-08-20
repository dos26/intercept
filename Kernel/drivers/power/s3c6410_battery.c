
#if defined (CONFIG_MACH_VINSQ)
#include "s3c6410_battery_vinsq.c"
#elif defined (CONFIG_MACH_VITAL)
#include "s3c6410_battery_vital.c"
#else
#include "s3c6410_battery_vinsq.c"
#endif

