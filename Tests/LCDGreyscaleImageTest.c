#include "LCDGreyscaleImageTest.h"
#include <avr/io.h>
#include "../LCDControl.h"
#include "../Time.h"

void GenerateCircle(LCDImageInfo* image)
{
	uint8_t radius = image->numberOfRows/2 + image->numberOfColumns/2;

	uint8_t cellValue;
	//populate the image with a circle
	for(int row = 0; row < image->numberOfRows; ++row)//regular int so we can have negative intermediate numbers for lhs
	{
		for(int column = 0; column < image->numberOfColumns; ++column)
		{
			cellValue = (row - image->numberOfRows/2)*(row - image->numberOfRows/2) + (column - image->numberOfColumns/2)*(column - image->numberOfColumns/2);
			//TODO::manipualte cell values using the radius
			image->imageMatrix[row*(image->numberOfColumns) + column] = 255 - cellValue;			
		}
	}
};


static volatile uint16_t frameCount = 0;
void LCDWriteFPS(uint8_t newSecond)
{
	if(newSecond)
	{
		LCDSetTextCursorPosition(0,64);
		LCDWriteSmallNumberAsString(frameCount);
		frameCount = 0;
		PORTC ^= (1<<PORTC5);
	}
	else
	{
		++frameCount;
	}
};


void TestGreyScaleLCD()
{
	
	DDRC |= (1<<PORTC5);
	
	FullTimerInit(16e6);
	LCDInit();
	
	
	LCDImageInfo circleImage = (LCDImageInfo){.numberOfColumns = 16, .numberOfRows = 24};
	char imageArray[circleImage.numberOfRows*circleImage.numberOfColumns];
	circleImage.imageMatrix = imageArray;
	GenerateCircle(&circleImage);   
					   
	char arrayOut[((circleImage.numberOfRows+7)/8)*circleImage.numberOfColumns];//we want to round up so we add (denominator-1) to the numerator
	
	
	char customChar[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,
						0b00000000,0b00000000,0b00000000,0b00000000,0b01111100,0b11111110,0b11111111,0b11111111,0b11111111,0b11111111,0b11111111,0b11111110,0b01111100,0b00000000,0b00000000,0b00000000,
						0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000001,0b00000001,0b00000001,0b00000001,0b00000001,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000};
	LCDImageInfo image2 = (LCDImageInfo){.numberOfRows = 3, .numberOfColumns = 16, .imageMatrix = customChar};

	LCDWriteArray(&image2,100, 2);
	
	const uint8_t tensOfMillisecondsPerSecond = 100;
	const uint8_t thresholdTop = 200;
	uint8_t currentTime, threshold = thresholdTop, timeCounter = tensOfMillisecondsPerSecond+1;
	
	LCDImageInfo denseCircleImage = (LCDImageInfo){.numberOfRows = (circleImage.numberOfRows + 7)/8, .numberOfColumns = circleImage.numberOfColumns, .imageMatrix = arrayOut};
	while(1)
	{
		currentTime = GetTimerValue(TensOfMilliSeconds);
		uint8_t newSecond = currentTime < timeCounter;//the time rolls over to 0 when it hits a second
		if(currentTime != timeCounter)
		{
			timeCounter = currentTime;
			ConvertToDenseArray(&circleImage, arrayOut, threshold);
			
			threshold = threshold == 250 ? thresholdTop : threshold+10;
		}
		LCDWriteFPS(newSecond);
		LCDWriteArray(&denseCircleImage,0, 0);
		
	}
};

