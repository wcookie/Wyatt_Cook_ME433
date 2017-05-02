/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


void init_IMU(){
  ANSELBbits.ANSB2 = 0;
  ANSELBbits.ANSB3 = 0;
   __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
  i2c_master_setup();                       // init I2C2, which we use as a master
  __builtin_enable_interrupts(); 
    char CTRL1_XL = 0b10000010;
    char CTRL2_G = 0b1000100;
    char CTRL3_C = 0b00000100;
    //accelerometer
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR<<1);
    i2c_master_send(0x10);
    i2c_master_send(CTRL1_XL);
    i2c_master_stop();
    //gyro
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR<<1);
    i2c_master_send(0x11);
    i2c_master_send(CTRL2_G);
    i2c_master_stop();
    //multi read
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR<<1);
    i2c_master_send(0x12);
    i2c_master_send(CTRL3_C);
    i2c_master_stop();
}

short combineNums(char * data, char index){
    short ret = data[index + 1] << 8;
    ret |= data[index];
    return ret;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    char data[14];
    short temp;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short accel_x;
    short accel_y;
    short accel_z;
    char message[20];
    char var;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
             init_IMU();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    
    
    //temp low is at 0x20
    //temp low, high
    //gyro:
    //x (l,h), y(l,h), z(l,h)
    //accel:
    //x (l,h), y(l,h), z(l,h)

    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0xF);
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1);
    var = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();


    _CP0_SET_COUNT(0);
            //12000 = the time
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
          while(_CP0_GET_COUNT() < 400000){
            ;
        }
        _CP0_SET_COUNT(0);
        int x, y;
         for (y=20; y < 28; ++ y){
            for (x = 60; x < 100; ++ x){
                LCD_drawPixel(x, y, BLACK);
            }
        }
         for (y=36; y < 44; ++ y){
            for (x = 60; x < 100; ++ x){
                LCD_drawPixel(x, y, BLACK);
            }
        }
         for (y=52; y < 60; ++ y){
            for (x = 60; x < 100; ++ x){
                LCD_drawPixel(x, y, BLACK);
            }
        }
         for (y=68; y < 76; ++ y){
            for (x = 60; x < 100; ++ x){
                LCD_drawPixel(x, y, BLACK);
            }
        }
        i2c_read_multiple(SLAVE_ADDR, 0x20, data, 14);
        temp = combineNums(data, 0);
        gyro_x = combineNums(data, 2);
        gyro_y = combineNums(data, 4);
        gyro_z = combineNums(data, 6);
        accel_x = combineNums(data, 8);
        accel_y = combineNums(data, 10);
        accel_z = combineNums(data, 12);
        sprintf(message, "accel x: %d", accel_x);
        display_string(message, 20, 20, CYAN);
        sprintf(message, "accel y: %d", accel_y);
        display_string(message, 36, 20, CYAN);
        sprintf(message, "gyro x: %d", gyro_x);
        display_string(message, 52, 20, CYAN);
        sprintf(message, "gyro y: %d:", gyro_y);
        display_string(message, 68, 20, CYAN);
        
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
