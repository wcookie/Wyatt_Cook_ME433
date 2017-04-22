#ifndef ILI9163C_H__
#define ILI9163C_H__

#include <ILI9163C.h>

void display_character(char c, unsigned short row, unsigned short col, unsigned short color);
void display_multiline_string(char msg[], unsigned short row, unsigned short col, unsigned short xGap, unsigned short yGap, unsigned short color);
void display_string(char msg[], unsigned short row, unsigned short col, unsigned short color);
void draw_bar(unsigned short startPos, unsigned short length, unsigned short row, unsigned short color);
