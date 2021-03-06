
#ifndef LCD_H__
#define LCD_H__

#include <stdint.h>

enum LCDChip
{
	CHIP1,
	CHIP2,
	NONE
};

enum Register
{
	RegisterINST,
	RegisterDATA
};

void LCDInit();

void LCDClear();

void LCDSetTextCursorPosition(uint8_t row, uint8_t col);

//padding is the amount of space to leave between characters. 1 is typically enough
void LCDWriteChar(uint8_t c, uint8_t invert, uint8_t padding);

void LCDWriteStr(const char* str, uint8_t invert);

void LCDWriteCustomChar(char customChar[5], int xPosition, int yPosition);

void LCDWriteByte(const char data, uint8_t xPosition, uint8_t yPosition);

void LCDWriteSmallNumberAsString(uint16_t number);

//this is super slow
uint8_t LCDRead(enum LCDChip chip, enum Register reg);


 typedef struct
 {
	char* imageMatrix; //this is a matrix of columns and is arranged smallendian. the rightmost bit of each byte will be the top of its column
	uint16_t numberOfColumns, numberOfRows;
 } LCDImageInfo;

//each row has a height of 8 bits. If the yPosition is not a multiple of 8 then the unset bits in the rows that are partially filled are set to 0
//columns are 1 bit wide while rows are 8 bits tall. the rightmost bit of each byte will be the top of its column
void LCDWriteArray(const LCDImageInfo* const image, uint8_t xPosition, uint8_t yPosition);

//This method converts the arrayIn to arrayOut.
//The contents of arrayIn are bytes that each represent a pixel. These bytes could hold color information or just be 1 and 0 for black white.
//The contents of arrayOut are bytes where each bit represents a pixel. The bytes are arranged smallendian.
//The threshold is the metric by which the bytes are converted, if the byte in arrayIn is equal or larger than the threshold then the pixel is on.
void ConvertToDenseArray(const LCDImageInfo* const image, char arrayOut[], uint8_t threshold);


#endif  /* LCD_H__ */