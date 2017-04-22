#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include <stdio.h>
#include "ILI9163C.h"
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

void display_character(char c, unsigned short row, unsigned short col, unsigned short color){
    short index = c - 32;
    unsigned short i = col;
    unsigned short j =0;
    for(i; i < (5 + col); ++i){
        for (j=0; j < 8; ++j){
            if ((ASCII[index][i - col] >> j) & 1){
                LCD_drawPixel(i, row + j, color);
            }
        }
    }
}

void display_multiline_string(char msg[], unsigned short row, unsigned short col, unsigned short xGap, unsigned short yGap, unsigned short color){
    int i = 0;
    while (msg[i]){
        if (msg[i] == '\n'){
            row += yGap;
        }
        else if (msg[i] == '\t'){
            col += 4 * xGap;
        }
        else{
            display_character(msg[i], row, col, color);
            col += xGap;
        }
        ++i;
    }
}
void display_string(char msg[], unsigned short row, unsigned short col, unsigned short color){
    int i = 0;
    unsigned short xGap = 5;
    while (msg[i]){
        display_character(msg[i], row, col, color);
        col += xGap;
        ++i;
    }
}

void draw_bar(unsigned short startPos, unsigned short length, unsigned short row, unsigned short color){
    unsigned short i = startPos;
    for (i; i < startPos + length; ++i){
        LCD_drawPixel(i, row, color);
        LCD_drawPixel(i, row + 1, color);
        LCD_drawPixel(i, row + 2, color);
        LCD_drawPixel(i, row + 3, color);
    }
}

int main() {
    SPI1_init();
    LCD_init();
    LCD_clearScreen(BLACK);
    //display_character('h', 20, 10, WHITE);
    char message[100];
    //sprintf(message, "Hi Daniel");
    //display_string(message, 40, 40, WHITE);
    unsigned short barStart = 0;
    unsigned short barLength = 20;
    unsigned short direction = 1;
    int integer = 0;
    _CP0_SET_COUNT(0);
    while (1){
        
        //LCD_clearScreen(BLACK);
        while(_CP0_GET_COUNT() <= 400000){
            ;
        }
        _CP0_SET_COUNT(0);
        //draw_bar(0, 128, 10, BLACK);
        sprintf(message, "Hello World! %d", integer);
        display_string(message, 32, 28, MAGENTA);
        draw_bar(0, integer, 10, CYAN);
        ++integer;
        if(integer > 128){
            integer = 0;
            draw_bar(0, 128, 10, BLACK);
        }
       
   }
}