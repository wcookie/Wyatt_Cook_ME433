#include <LCD.h>
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