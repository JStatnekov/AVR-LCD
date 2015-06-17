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

void LCDWriteChar(uint8_t ch);

void LCDWriteStr(const char* str);

void LCDWriteCustomChar(char customChar[5], int xPosition, int yPosition);

uint8_t LCDRead(enum LCDChip chip, enum Register reg);

void LCDWriteByte(const char data, uint8_t xPosition, uint8_t yPosition);



 typedef struct
 {
	const char* imageMatrix;
	uint8_t numberOfColumns, numberOfRows;
 } LCDImageInfo;

//each row has a height of 8 bits. If the yPosition is not a multiple of 8 then the unset bits in the rows that are partially filled 
//are set to 0
//columns are 1 bit wide while rows are 8 bits long, this is because a single byte will be displayed smallendian from top down along a column
void LCDWriteArray(const LCDImageInfo* const image, uint8_t xPosition, uint8_t yPosition);

//This method converts the arrayIn to arrayOut.
//The contents of arrayIn are bytes that each represent a pixel. These bytes could hold color information or just be 1 and 0 for black white.
//The contents of arrayOut are bytes where each bit represents a pixel. The bytes are arranged smallendian from top down along a column.
//The threshold is the metric by which the bytes are converted, if the byte in arrayIn is equal or larger than the threshold then the pixel is on.
void ConvertToDenseArray(const char arrayIn[], char arrayOut[], uint8_t arrayInNumberOfColumns, uint8_t ArrayInNumberOfRows, uint8_t threshold);

//create a method to overlay one matrix over another
//void overlay(backgroundMarix,

#endif  /* LCD_H__ */