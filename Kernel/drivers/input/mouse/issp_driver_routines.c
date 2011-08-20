// filename: ISSP_Driver_Routines.c
#include "issp_revision.h"
#ifdef PROJECT_REV_304
// Copyright 2006-2007, Cypress Semiconductor Corporation.
//
// This software is owned by Cypress Semiconductor Corporation (Cypress)
// and is protected by and subject to worldwide patent protection (United
// States and foreign), United States copyright laws and international
// treaty provisions. Cypress hereby grants to licensee a personal,
// non-exclusive, non-transferable license to copy, use, modify, create
// derivative works of, and compile the Cypress Source Code and derivative
// works for the sole purpose of creating custom software in support of
// licensee product to be used only in conjunction with a Cypress integrated
// circuit as specified in the applicable agreement. Any reproduction,
// modification, translation, compilation, or representation of this
// software except as specified above is prohibited without the express
// written permission of Cypress.
//
// Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,
// WITH REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
// Cypress reserves the right to make changes without further notice to the
// materials described herein. Cypress does not assume any liability arising
// out of the application or use of any product or circuit described herein.
// Cypress does not authorize its products for use as critical components in
// life-support systems where a malfunction or failure may reasonably be
// expected to result in significant injury to the user. The inclusion of
// Cypressï¿?product in a life-support systems application implies that the
// manufacturer assumes all risk of such use and in doing so indemnifies
// Cypress against all charges.
//
// Use may be limited by and subject to the applicable Cypress software
// license agreement.
//
//--------------------------------------------------------------------------

//#include <m8c.h>        // part specific constants and macros
//#include "psocapi.h"    // PSoC API definitions for all User Modules

#include "issp_defs.h"
#include "issp_errors.h"
#include "issp_directives.h"


//#include "Device.h"

//#include <intrins.h>
#include "firmware_update.h" //etinum.LJ.firmware_update
#include <linux/delay.h> //etinum.LJ.firmware_update


#define SECURITY_DATA	0xFF

extern    unsigned char    bTargetDataPtr;
extern    unsigned char    abTargetDataOUT[TARGET_DATABUFF_LEN];


// ****************************** PORT BIT MASKS ******************************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
#define SDATA_PIN   0x10        // P1.4
#define SCLK_PIN    0x08        // P1.3
#define XRES_PIN    0x40        // P2.6
#define TARGET_VDD  0x40        // P2.6


// ((((((((((((((((((((((( DEMO ISSP SUBROUTINE SECTION )))))))))))))))))))))))
// ((((( Demo Routines can be deleted in final ISSP project if not used   )))))
// ((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))))

// ============================================================================
// InitTargetTestData()
// !!!!!!!!!!!!!!!!!!FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!
// PROCESSOR_SPECIFIC
// Loads a 64-Byte array to use as test data to program target. Ultimately,
// this data should be fed to the Host by some other means, ie: I2C, RS232,
// etc. Data should be derived from hex file.
//  Global variables affected:
//    bTargetDataPtr
//    abTargetDataOUT
// ============================================================================
void InitTargetTestData(unsigned char bBlockNum, unsigned char bBankNum)
{
    // create unique data for each block
    for (bTargetDataPtr = 0; bTargetDataPtr < TARGET_DATABUFF_LEN; bTargetDataPtr++)
    {
        abTargetDataOUT[bTargetDataPtr] = 0x55;
    }
}


// ============================================================================
// LoadArrayWithSecurityData()
// !!!!!!!!!!!!!!!!!!FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!
// PROCESSOR_SPECIFIC
// Most likely this data will be fed to the Host by some other means, ie: I2C,
// RS232, etc., or will be fixed in the host. The security data should come
// from the hex file.
//   bStart  - the starting byte in the array for loading data
//   bLength - the number of byte to write into the array
//   bType   - the security data to write over the range defined by bStart and
//             bLength
// ============================================================================
void LoadArrayWithSecurityData(unsigned char bStart, unsigned char bLength, unsigned char bType)
{
    // Now, write the desired security-bytes for the range specified
    for (bTargetDataPtr = bStart; bTargetDataPtr < bLength; bTargetDataPtr++)
    {
        if (bTargetDataPtr < (bLength/4) )
            abTargetDataOUT[bTargetDataPtr] = bType;
        else
            abTargetDataOUT[bTargetDataPtr] = 0x00;
    }
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// Delay()
// This delay uses a simple "nop" loop. With the CPU running at 24MHz, each
// pass of the loop is about 1 usec plus an overhead of about 3 usec.
//      total delay = (n + 3) * 1 usec
// To adjust delays and to adapt delays when porting this application, see the
// ISSP_Delays.h file.
// ****************************************************************************
void Delay(unsigned int n)
{
#if 1 //etinum.LJ.firmware_update
    udelay(n);
#else
    // TODO: microsec delay
    while(n)
    {
        asm("nop");
        n -= 1;
    }
#endif
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// LoadProgramData()
// The final application should load program data from HEX file generated by
// PSoC Designer into a 64 byte host ram buffer.
//    1. Read data from next line in hex file into ram buffer. One record
//      (line) is 64 bytes of data.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread data from file or abort
//    4. Exit this Function and Program block or verify the block.
// This demo program will, instead, load predetermined data into each block.
// The demo does it this way because there is no comm link to get data.
// ****************************************************************************
void LoadProgramData(unsigned char bBlockNum, unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function InitTargetTestData fills buffer for demo
    InitTargetTestData(bBlockNum, bBankNum);

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns VOID.
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fLoadSecurityData()
// Load security data from hex file into 64 byte host ram buffer. In a fully
// functional program (not a demo) this routine should do the following:
//    1. Read data from security record in hex file into ram buffer.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread security data from file or abort
//    4. Exit this Function and Program block
// In this demo routine, all of the security data is set to unprotected (0x00)
// and it returns.
// This function always returns PASS. The flag return is reserving
// functionality for non-demo versions.
// ****************************************************************************
signed char fLoadSecurityData(unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function LoadArrayWithSecurityData fills buffer for demo
//    LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0x00);
    LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, SECURITY_DATA);		//PTJ: 0x1B (00 01 10 11) is more interesting security data than 0x00 for testing purposes

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns PASS.
    return(PASS);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fSDATACheck()
// Check SDATA pin for high or low logic level and return value to calling
// routine.
// Returns:
//     0 if the pin was low.
//     1 if the pin was high.
// ****************************************************************************
unsigned char fSDATACheck(void)
{
    //etinum.LJ.firmware_update if(PRT1DR & SDATA_PIN)
        
    if(g_cypress_fwdata->get_data()) //etinum.LJ.firmware_update //if (SDATA_Read())
        return(1);
    else
        return(0);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKHigh()
// Set the SCLK pin High
// ****************************************************************************
void SCLKHigh(void)
{
    //etinum.LJ.firmware_update PRT1DR |= SCLK_PIN;
    g_cypress_fwdata->set_sclk(1); //etinum.LJ.firmware_update //SCLK_Write(1);
    Delay(4);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKLow()
// Make Clock pin Low
// ****************************************************************************
void SCLKLow(void)
{
    //etinum.LJ.firmware_update PRT1DR &= ~SCLK_PIN;
    g_cypress_fwdata->set_sclk(0); //etinum.LJ.firmware_update //SCLK_Write(0);
    Delay(4);
}

#ifndef RESET_MODE  // Only needed for power cycle mode
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKHiZ()
// Set SCLK pin to HighZ drive mode.
// ****************************************************************************
void SetSCLKHiZ(void)
{
    //etinum.LJ.firmware_update PRT1DM0 &= ~SCLK_PIN;
    //PRT1DM1 |=  SCLK_PIN;
    //PRT1DM2 &= ~SCLK_PIN;
    g_cypress_fwdata->set_drive(0, 0); //etinum.LJ.firmware_update //SCLK_SetDriveMode(SCLK_DM_DIG_HIZ);
}
#endif

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKStrong()
// Set SCLK to an output (Strong drive mode)
// ****************************************************************************
void SetSCLKStrong(void)
{
    //etinum.LJ.firmware_update PRT1DM0 |=  SCLK_PIN;
    //PRT1DM1 &= ~SCLK_PIN;
    //PRT1DM2 &= ~SCLK_PIN;
    g_cypress_fwdata->set_drive(0, 1); //etinum.LJ.firmware_update //SCLK_SetDriveMode(SCLK_DM_STRONG);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHigh()
// Make SDATA pin High
// ****************************************************************************
void SetSDATAHigh(void)
{
    //etinum.LJ.firmware_update PRT1DR |= SDATA_PIN;
    g_cypress_fwdata->set_data(1); //etinum.LJ.firmware_update //SDATA_Write(1);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATALow()
// Make SDATA pin Low
// ****************************************************************************
void SetSDATALow(void)
{
    //etinum.LJ.firmware_update PRT1DR &= ~SDATA_PIN;
    g_cypress_fwdata->set_data(0); //etinum.LJ.firmware_update //SDATA_Write(0);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHiZ()
// Set SDATA pin to an input (HighZ drive mode).
// ****************************************************************************
void SetSDATAHiZ(void)
{
    //etinum.LJ.firmware_update PRT1DM0 &= ~SDATA_PIN;
    //PRT1DM1 |=  SDATA_PIN;
    //PRT1DM2 &= ~SDATA_PIN;
    g_cypress_fwdata->set_drive(1, 0); //etinum.LJ.firmware_update //SDATA_SetDriveMode(SDATA_DM_DIG_HIZ);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAStrong()
// Set SDATA for transmission (Strong drive mode) -- as opposed to being set to
// High Z for receiving data.
// ****************************************************************************
void SetSDATAStrong(void)
{
    //etinum.LJ.firmware_update PRT1DM0 |=  SDATA_PIN;
    //PRT1DM1 &= ~SDATA_PIN;
    //PRT1DM2 &= ~SDATA_PIN;
    g_cypress_fwdata->set_drive(1, 1); //etinum.LJ.firmware_update //SDATA_SetDriveMode(SDATA_DM_STRONG);
}

#ifdef RESET_MODE
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetXRESStrong()
// Set external reset (XRES) to an output (Strong drive mode).
// ****************************************************************************
void SetXRESStrong(void)
{
    PRT2DM0 |=  XRES_PIN;
    PRT2DM1 &= ~XRES_PIN;
    PRT2DM2 &= ~XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// AssertXRES()
// Set XRES pin High
// ****************************************************************************
void AssertXRES(void)
{
    PRT2DR |= XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// DeassertXRES()
// Set XRES pin low.
// ****************************************************************************
void DeassertXRES(void)
{
    PRT2DR &= ~XRES_PIN;
}
#else

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetTargetVDDStrong()
// Set VDD pin (PWR) to an output (Strong drive mode).
// ****************************************************************************
void SetTargetVDDStrong(void)
{
    //etinum.LJ.firmware_update PRT2DM0 |=  TARGET_VDD;
    //PRT2DM1 &= ~TARGET_VDD;
    //PRT2DM2 &= ~TARGET_VDD;
    g_cypress_fwdata->set_vdd(0);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// ApplyTargetVDD()
// Provide power to the target PSoC's Vdd pin through a GPIO.
// ****************************************************************************
void ApplyTargetVDD(void)
{
    //etinum.LJ.firmware_update PRT2DR |= TARGET_VDD;
    g_cypress_fwdata->set_vdd(1); //etinum.LJ.firmware_update //Vdd_Write(1);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// RemoveTargetVDD()
// Remove power from the target PSoC's Vdd pin.
// ****************************************************************************
void RemoveTargetVDD(void)
{
    //etinum.LJ.firmware_update PRT2DR &= ~TARGET_VDD;
    g_cypress_fwdata->set_vdd(0); //etinum.LJ.firmware_update //Vdd_Write(0);
}
#endif
//end of file ISSP_Drive_Routines.c

#endif  //(PROJECT_REV_)
//end of file ISSP_Drive_Routines.c
