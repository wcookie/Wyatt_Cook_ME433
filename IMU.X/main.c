#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include <stdio.h>
#include "i2c_master_noint.h"
#include "LCD.h"
// DEVCFG0

#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow multiple reconfigurations
#pragma config IOL1WAY = ON// allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module



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

int main(){
    init_IMU();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    char data[14];
    
    //temp low is at 0x20
    //temp low, high
    //gyro:
    //x (l,h), y(l,h), z(l,h)
    //accel:
    //x (l,h), y(l,h), z(l,h)
    short temp;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short accel_x;
    short accel_y;
    short accel_z;
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0xF);
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1);
    char var = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    char message[20];
    
    _CP0_SET_COUNT(0);
    while(1){
        while(_CP0_GET_COUNT() < 40000){
            ;
        }
        _CP0_SET_COUNT(0);
        i2c_read_multiple(SLAVE_ADDR, 0x20, data, 14);
        temp = combineNums(data, 0);
        gyro_x = combineNums(data, 2);
        gyro_y = combineNums(data, 4);
        gyro_z = combineNums(data, 6);
        accel_x = combineNums(data, 8);
        accel_y = combineNums(data, 10);
        accel_z = combineNums(data, 12);
        sprintf(message, "gyro: %d", gyro_x);
        display_string(message, 20, 20, CYAN);
        int x, y;
         for (y=20; y < 28; ++ y){
            for (x = 50; x < 70; ++ x){
                LCD_drawPixel(x, y, BLACK);
            }
        }
        
    }
    
    
}