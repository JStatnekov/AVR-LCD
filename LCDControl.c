#include "LCDControl.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "font.h"

//The following Data Direction Registers are the channels for the data bits which come from pins 4-11 on the LCD
#define Data0DataDirectionRegister DDRB
#define DataPin0 PINB
#define DataPort0 PORTB
#define DataPortNumber0 PORTB5

#define Data1DataDirectionRegister DDRD
#define DataPin1 PIND
#define DataPort1 PORTD
#define DataPortNumber1 PORTD1

#define Data2DataDirectionRegister DDRD
#define DataPin2 PIND
#define DataPort2 PORTD
#define DataPortNumber2 PORTD2

#define Data3DataDirectionRegister DDRD
#define DataPin3 PIND
#define DataPort3 PORTD
#define DataPortNumber3 PORTD3

#define Data4DataDirectionRegister DDRD
#define DataPin4 PIND
#define DataPort4 PORTD
#define DataPortNumber4 PORTD4

#define Data5DataDirectionRegister DDRD
#define DataPin5 PIND
#define DataPort5 PORTD
#define DataPortNumber5 PORTD5

#define Data6DataDirectionRegister DDRD
#define DataPin6 PIND
#define DataPort6 PORTD
#define DataPortNumber6 PORTD6

#define Data7DataDirectionRegister DDRD
#define DataPin7 PIND
#define DataPort7 PORTD
#define DataPortNumber7 PORTD7



#define CSDataDirectionRegister DDRB
#define CSPorts PORTB
#define CS1PortNumber PORTB1
#define CS2PortNumber PORTB0

#define RWDataDirectionRegister DDRB
#define RWPort PORTB
#define RWPortNumber PORTB2

#define DIDataDirectionRegister DDRC
#define DIPort PORTC
#define DIPortNumber PORTC0

#define EDataDirectionRegister DDRC
#define EPort PORTC
#define EPortNumber PORTC1



#define LCD_SELECT_READ {RWPort |= (1<<RWPortNumber); \
Data0DataDirectionRegister &= ~(1<<DataPortNumber0); \
Data1DataDirectionRegister &= ~(1<<DataPortNumber1); \
Data2DataDirectionRegister &= ~(1<<DataPortNumber2); \
Data3DataDirectionRegister &= ~(1<<DataPortNumber3); \
Data4DataDirectionRegister &= ~(1<<DataPortNumber4); \
Data5DataDirectionRegister &= ~(1<<DataPortNumber5); \
Data6DataDirectionRegister &= ~(1<<DataPortNumber6); \
Data7DataDirectionRegister &= ~(1<<DataPortNumber7);}

#define LCD_SELECT_WRITE {RWPort &= ~(1<<RWPortNumber); \
Data0DataDirectionRegister |= (1<<DataPortNumber0); \
Data1DataDirectionRegister |= (1<<DataPortNumber1); \
Data2DataDirectionRegister |= (1<<DataPortNumber2); \
Data3DataDirectionRegister |= (1<<DataPortNumber3); \
Data4DataDirectionRegister |= (1<<DataPortNumber4); \
Data5DataDirectionRegister |= (1<<DataPortNumber5); \
Data6DataDirectionRegister |= (1<<DataPortNumber6); \
Data7DataDirectionRegister |= (1<<DataPortNumber7);}

#define LCD_SELECT_CS1 (CSPorts |= (1<<CS2PortNumber), CSPorts &= ~(1<<CS1PortNumber))
#define LCD_SELECT_CS2 (CSPorts |= (1<<CS1PortNumber), CSPorts &= ~(1<<CS2PortNumber))
#define LCD_E_HIGH (EPort |= (1<<EPortNumber))
#define LCD_E_LOW (EPort &= ~(1<<EPortNumber))
#define LCD_SELECT_INST (DIPort &= ~(1<<DIPortNumber))
#define LCD_SELECT_DATA (DIPort |= (1<<DIPortNumber))
//The following values come directly from the datasheet
#define LCD_YADDR(Y) (((Y) & 0b00111111) | 0b01000000)
#define LCD_XADDR(X) (((X) & 0b00000111) | 0b10111000)
#define LCD_STARTLINE(L) (((L) & 0b00111111) | 0b11000000)




void LCD_SELECT_CHIP(enum LCDChip chip)
{ 
	if(chip == CHIP1)
		LCD_SELECT_CS1;
	else
		LCD_SELECT_CS2;
};

void LCD_SELECT_REG(enum Register reg)
{
	if(reg == RegisterDATA)
		LCD_SELECT_DATA;
	else
		LCD_SELECT_INST;
}

// These might need to be adjusted.  If they're too low you'll get garbled
// data; too high and updates will be slow.
static uint8_t lcd_dl;
#define LCD_SHORT_DELAY for(lcd_dl=0; lcd_dl<2; ++lcd_dl){ asm volatile("nop\n"); }
#define LCD_LONG_DELAY for(lcd_dl=0; lcd_dl<8; ++lcd_dl){ asm volatile("nop\n"); }

static enum LCDChip cache_chip = NONE;

static uint8_t textCursorX;
static uint8_t textCursorY;


void LCDWriteData(enum LCDChip chip, enum Register reg, uint8_t data) {
  LCD_SELECT_WRITE;
  LCD_SELECT_CHIP(chip);
  LCD_SELECT_REG(reg);

//select the bit of the data that is desired for this port
//then 
	DataPort0 = (data & 1) == 0 ? DataPort0 & ~(1<<DataPortNumber0) : DataPort0 | (1<<DataPortNumber0);
	DataPort1 = (data & (1<<1)) == 0 ? DataPort1 & ~(1<<DataPortNumber1) : DataPort1 | (1<<DataPortNumber1);
	DataPort2 = (data & (1<<2)) == 0 ? DataPort2 & ~(1<<DataPortNumber2) : DataPort2 | (1<<DataPortNumber2);
	DataPort3 = (data & (1<<3)) == 0 ? DataPort3 & ~(1<<DataPortNumber3) : DataPort3 | (1<<DataPortNumber3);
	DataPort4 = (data & (1<<4)) == 0 ? DataPort4 & ~(1<<DataPortNumber4) : DataPort4 | (1<<DataPortNumber4);
	DataPort5 = (data & (1<<5)) == 0 ? DataPort5 & ~(1<<DataPortNumber5) : DataPort5 | (1<<DataPortNumber5);
	DataPort6 = (data & (1<<6)) == 0 ? DataPort6 & ~(1<<DataPortNumber6) : DataPort6 | (1<<DataPortNumber6);
	DataPort7 = (data & (1<<7)) == 0 ? DataPort7 & ~(1<<DataPortNumber7) : DataPort7 | (1<<DataPortNumber7);

  LCD_LONG_DELAY;
  LCD_E_HIGH;
  LCD_SHORT_DELAY;
  LCD_E_LOW;
};

//this is super slow and should be avoided
uint8_t LCDRead(enum LCDChip chip, enum Register reg) {
	uint8_t d;
	LCD_SELECT_READ;
	LCD_SELECT_CHIP(chip);
	LCD_SELECT_REG(reg);
	LCD_LONG_DELAY;
	LCD_E_HIGH;
	LCD_SHORT_DELAY;

	d = (DataPin0 & (1<<DataPortNumber0)) |
		(DataPin1 & (1<<DataPortNumber1)) |
		(DataPin2 & (1<<DataPortNumber2)) |
		(DataPin3 & (1<<DataPortNumber3)) |
		(DataPin4 & (1<<DataPortNumber4)) |
		(DataPin5 & (1<<DataPortNumber5)) |
		(DataPin6 & (1<<DataPortNumber6)) |
		(DataPin7 & (1<<DataPortNumber7));

	LCD_E_LOW;
	return d;
};

void LCDWait(enum LCDChip chip) 
{
//RESET CODE iS 00001000 
//BUSY CODE is  01000000
	while(LCDRead(chip, RegisterINST) & ((1<<7) | (1<<4))) { }
};

void LCDWriteWait(enum LCDChip chip, enum Register reg, uint8_t data) 
{ 
  LCDWriteData(chip, reg, data);

  LCDWait(chip);
};

void LCDInit() {

	Data0DataDirectionRegister &= ~(1<<DataPortNumber0);
	Data1DataDirectionRegister &= ~(1<<DataPortNumber1);
	Data2DataDirectionRegister &= ~(1<<DataPortNumber2);
	Data3DataDirectionRegister &= ~(1<<DataPortNumber3);
	Data4DataDirectionRegister &= ~(1<<DataPortNumber4);
	Data5DataDirectionRegister &= ~(1<<DataPortNumber5);
	Data6DataDirectionRegister &= ~(1<<DataPortNumber6);
	Data7DataDirectionRegister &= ~(1<<DataPortNumber7);
	//DDRB   =  0x00;  // PORTB inputs for now.

	CSDataDirectionRegister |= ((1<<CS1PortNumber) | (1<< CS2PortNumber));
	DIDataDirectionRegister |= (1<<DIPortNumber);
	RWDataDirectionRegister |= (1<<RWPortNumber);
	EDataDirectionRegister |= (1<<EPortNumber);
	//DDRD  |=  0xEC;  // 5 outputs on PORTD

	RWPort &= ~(1<<RWPortNumber);
	DIPort &= ~(1<<DIPortNumber);
	EPort &= ~(1<<DIPortNumber); //PORTD &= ~0xE0;  // R/W, D/I, E low

	CSPorts |= ((1<<CS1PortNumber) | (1 << CS2PortNumber));  //PORTD |=  0x0C;  // CS1, CS2 high

	for(uint16_t d=0;d<5000; ++d);  // let the above sink in a bit.

	LCDWait(CHIP1); 
	LCDWait(CHIP2);

	LCDWriteWait(CHIP1, RegisterINST, 0b00111111);	
	LCDWriteWait(CHIP2, RegisterINST, 0b00111111);//bad line?

	LCDWriteWait(CHIP1, RegisterINST, LCD_STARTLINE(0));
	LCDWriteWait(CHIP2, RegisterINST, LCD_STARTLINE(0));

	LCDClear();
};

void ClearChip(enum LCDChip chip)
{
	uint8_t y;
	for(uint8_t x = 0; x < 8; ++x) {
		LCDWriteWait(chip, RegisterINST, LCD_YADDR(0));
		LCDWriteWait(chip, RegisterINST, LCD_XADDR(x));
		for(y = 0;y < 64; ++y) {
			LCDWriteWait(chip, RegisterDATA, 0);
		}
	}
};

void LCDClear() {
	ClearChip(CHIP1);
	ClearChip(CHIP2);
	cache_chip = NONE;
};

void LCDSetTextCursorPosition(uint8_t row, uint8_t col) {
	textCursorX = 6 * col;
	textCursorY = 8 * row;
};

void LCDWriteChar(uint8_t c, uint8_t invert, uint8_t padding)
{
	if (c < 32 || c > 128) return;
	
	uint8_t characterWidth = 5;
	const uint8_t* charPointer = font_5x7_data + (characterWidth * (c-32));
	uint8_t columnData;
	for(uint8_t charColumnNum = 0; charColumnNum < characterWidth; ++charColumnNum) { //for each column of data
		columnData = pgm_read_byte(charPointer + charColumnNum); //read the rows worth of data
		LCDWriteByte((invert ? ~columnData : columnData), textCursorX + charColumnNum, textCursorY);
	}

	for(uint8_t paddingColumn = 0; paddingColumn < padding ; paddingColumn++ )
	{
		LCDWriteByte((invert ? 0b11111111 : 0), textCursorX + characterWidth + paddingColumn, textCursorY);
	}
	
	textCursorX += (5+padding);
};


void LCDWriteStr(const char* str, uint8_t invert) {
	while(*str) {
		LCDWriteChar(*str++, invert, 1);
	}
};

void LCDWriteCustomChar(char customChar[5], int xPosition, int yPosition)
{
	enum LCDChip chip = (xPosition > 63) ? CHIP2: CHIP1;
	
	//the y direction is horizontal, the x is vertical, 0,0 is upper left
	uint8_t lcd_x = ((yPosition) & 0b00111111) >> 3;
	
	for(uint8_t i = 0; i < 5; ++i)
	{
		uint8_t lcd_y = ((xPosition+i) & 0b00111111);
		LCDWriteWait(chip, RegisterINST, LCD_YADDR(lcd_y));
		LCDWriteWait(chip, RegisterINST, LCD_XADDR(lcd_x));
		LCDWriteWait(chip, RegisterDATA, customChar[i]);
	}
};

void LCDWriteByte(const char data, uint8_t xPosition, uint8_t yPosition)
{
	enum LCDChip chip = (xPosition > 63) ? CHIP2 : CHIP1;
	uint8_t verticalShift = yPosition % 8;//this is how much we are shifting an entry because it is not aligned with the bits that make up a column
	uint8_t extraRowRequired = verticalShift != 0;
		
	//the y direction is horizontal, the x is vertical, upper left is 0,0
	LCDWriteWait(chip, RegisterINST, LCD_YADDR(xPosition & 63));//we only want to write to the addressable memory space
	LCDWriteWait(chip, RegisterINST, LCD_XADDR(yPosition/8));//we're relying on truncation
	
	LCDWriteWait(chip, RegisterDATA, (data << verticalShift));	
	
	if(extraRowRequired)
	{
		uint8_t verticalShiftComplement = 8 - verticalShift;		
		LCDWriteWait(chip, RegisterINST, LCD_YADDR(xPosition & 63));//we only want to write to the addressable memory space
		LCDWriteWait(chip, RegisterINST, LCD_XADDR(1+(yPosition/8)));//we're relying on truncation
		
		LCDWriteWait(chip, RegisterDATA, (data >> verticalShiftComplement));//get only the bits that are on this line from the current position in the array
	}
};



void LCDWriteSmallNumberAsString(uint16_t number)
{
	uint8_t ones = (number%10)+'0';//cool hack bro
	uint8_t tens = ((number%100)/10)+'0';
	uint8_t hundreds = ((number%1000)/100)+'0';
	uint8_t thousands = ((number%10000)/1000)+'0';
	uint8_t tenThousands = ((number%100000)/10000)+'0';
	
	LCDWriteChar(tenThousands,0,1);
	LCDWriteChar(thousands,0,1);
	LCDWriteChar(hundreds,0,1);
	LCDWriteChar(tens,0,1);
	LCDWriteChar(ones,0,1);
};


void LCDWriteArray(const LCDImageInfo* const image, uint8_t xPosition, uint8_t yPosition)
{
	uint8_t verticalShift = yPosition % 8;//this is how much we are shifting an entry because it is not aligned with the bits that make up a column
	
	uint8_t startingRow = yPosition/8; //position via the rowNumber, yes truncation in action
	enum LCDChip chip;
	
	for(uint8_t row = 0; row < image->numberOfRows; ++row)
	{
		for(uint8_t col = 0; col < image->numberOfColumns; ++col)
		{
			uint8_t xLocation = col + xPosition, yLocation = startingRow + row;			
			
			if(xLocation > 63)
			{
				chip = CHIP2;				
				LCDWriteWait(chip, RegisterINST, LCD_YADDR((xLocation - 64) & 63));
			}
			else
			{
				chip = CHIP1;				
				LCDWriteWait(chip, RegisterINST, LCD_YADDR(xLocation & 63));
			}					
			LCDWriteWait(chip, RegisterINST, LCD_XADDR(yLocation));
			
			//the right is the top
			//0b10101010 << 1 = 01010100
			uint8_t value = image->imageMatrix[row*image->numberOfColumns + col] << verticalShift;
			
			uint8_t extraRowRequired = verticalShift > 0;
			if(extraRowRequired && row > 0)
			{
				//this is a later row and we need to shift the previous row onto the top of this one
				value |= image->imageMatrix[(row-1)*image->numberOfColumns + col] >> (8-verticalShift);
			}
			LCDWriteWait(chip, RegisterDATA, value);
				
			if(extraRowRequired)
			{
				if(xLocation > 63)
				{
					LCDWriteWait(chip, RegisterINST, LCD_YADDR((xLocation - 64) & 63));
				}
				else
				{
					LCDWriteWait(chip, RegisterINST, LCD_YADDR(xLocation & 63));
				}
				LCDWriteWait(chip, RegisterINST, LCD_XADDR(yLocation+1));
					//
				////0b10101010 >> 7 = 00000001
				uint8_t additionalLineValue = (image->imageMatrix[row*image->numberOfColumns + col]) >> (8-verticalShift);
				//if(row+1 < image->numberOfRows)
				//{
					////0b01010101 << 1 = 10101010
					//additionalLineValue |= (image->imageMatrix[(row+1)*image->numberOfColumns + col]) << verticalShift;
					//LCDWriteSmallNumberAsString(additionalLineValue);
					//DelaySeconds(1);
					//
					LCDWriteWait(chip, RegisterDATA, additionalLineValue);
				//}
			}
		}
	}
}

void ConvertToDenseArray(const LCDImageInfo* const image, char arrayOut[], uint8_t threshold)
{
	//the dense array has the same number of columns but will have 1/8th the number of rows because we'll pack each input cell into a pixel
	uint16_t sizeOfOutputArray = image->numberOfColumns * ((image->numberOfRows + 7)/8);//we want to round up here
	uint16_t position;
	for(position = 0; position < sizeOfOutputArray; position++)
	{
		arrayOut[position] = 0;
	}
	
	//there are 8 rows and 128 columns and each position can hold 8 bits so the max value is 8*8*128 = 8192 so a 16bit int is necessary
	//however this max size is many times larger than the SRAM memory of the atmega328 so external ram is required for max size
	//even a 3 row by 24 column requires 8*3*24=576 byte array so a 16bit int is required	
	uint16_t rowOffset, denseRowOffset;
	uint8_t columnNumber, denseColumnBitPosition, value;
	for(uint8_t rowNumber = 0; rowNumber < image->numberOfRows; rowNumber++)
	{
		denseColumnBitPosition = rowNumber%8;
		rowOffset = rowNumber*image->numberOfColumns;
		denseRowOffset = (rowNumber/8)*image->numberOfColumns;
		
		for(columnNumber = 0; columnNumber < image->numberOfColumns; columnNumber++)
		{
			value = (image->imageMatrix[rowOffset + columnNumber] <= threshold);//TODO reverse
			
			arrayOut[denseRowOffset + columnNumber] |= (value << denseColumnBitPosition);
		}
	}
};