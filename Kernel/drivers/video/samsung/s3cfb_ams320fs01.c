
#if defined (CONFIG_MACH_VITAL)
#include "s3cfb_vital.c"
#elif defined (CONFIG_MACH_VINSQ)
#include "s3cfb_vinsq.c"
#else
#error "make sure your target source is setup!"
#include "s3cfb_vinsq.c"
#endif

