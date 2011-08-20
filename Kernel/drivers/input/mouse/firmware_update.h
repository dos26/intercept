#ifndef _FIRMWARE_UPDATE_H_
#define _FIRMWARE_UPDATE_H_

#define LJ_FIRMWARE_UPDATE

#ifdef LJ_FIRMWARE_UPDATE
#include <linux/types.h>

struct firmware_update_struct
{
    int updating;
    char* binary;
    size_t binsize;
    char version;
    void (*set_sclk)(int set);
    void (*set_data)(int set);
    int (*get_data)(void);
    void (*set_drive)(int clk_data_sel, int drive_mode); //0:clock 1:data, 0: high impedence 1:strong
    void (*set_vdd)(int set);
};

extern int cypress_update(struct firmware_update_struct* fwdata);
extern struct firmware_update_struct* g_cypress_fwdata;

#endif

#endif /* _FIRMWARE_UPDATE_H_ */
