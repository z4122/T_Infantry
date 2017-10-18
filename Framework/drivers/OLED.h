#ifndef __OLED_H__
#define __OLED_H__

/* exact-width signed integer types */
typedef   signed          char s8;
typedef   signed short     int s16;

/* exact-width unsigned integer types */
typedef unsigned          char u8;
typedef unsigned short     int u16;
// ===== APIs =====
// Notation:  this Oled has 8 rows (0-7)  and 21 columns (0-20)

  // put a String at row x and begins at column y
  // x = 0 ~ 7 ; y = 0 ~ (20 - sizeof(ch[]))  ; ch[] is your string
void Oled_Putstr(u8 x,u8 y,u8 ch[]);

  // put a Number at row x and begins at column y
  // x = 0 ~ 7 ; y = 0 ~ 12 ; c is your num of type signed 16-bit integer
void Oled_Putnum(u8 x,u8 y,s16 c);

  // clear 
void Oled_Clear(void);

  // init
void Oled_Init(void);


#endif