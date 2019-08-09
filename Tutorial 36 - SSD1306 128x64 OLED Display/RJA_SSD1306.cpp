/*
 * OLED_ClassV1.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: ASUS
 */

#include "RJA_SSD1306.h"

constexpr unsigned char OLED::font1[];

OLED::OLED()
{
	// TODO Auto-generated constructor stub

}

OLED::~OLED()
{
	// TODO Auto-generated destructor stub
}

void OLED::init(I2C_HandleTypeDef *hi2c1)
{
	uint8_t I2CBuff[2];
		//uint16_t OLEDCAddress = 0x78;
		hi2cI = hi2c1;

		//display off
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0xAE;//0xAF; //0xAE; //off
		//memory mode: horizontal addressing
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0x20;	//memory mode
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0x00;	//horizontal addressing
		//scan starting line
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0x40; //0x40;	//start line 0-63: 11XXXXXX. So 40h-7Fh (basically negative, i.e. go up by)
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0x00; //0x00;	//start line: horizontal? the above is vertical.
		//charge pump on (2 commands)
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0x8D;
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0x14;
		//entire display on, from RAM content
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0xA4;//0xA5; //all-on //0xA4;
		//display on
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);
		I2CBuff[0] = 0x00;
		I2CBuff[1] = 0xAF;//0xAF; //0xAE; //off
		HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 10);

		//initialise frame to zeros i.e. "empty" GDDRAM
		for(int r = 0; r < 64; r++)
		{
			for(int c = 0; c < 128; c++)
			{
				//pixel(c,r,0);	//problem!
				frame[c][r] = 0;
			}
		}
}

void OLED::drawFullscreen() //I2C_HandleTypeDef *hi2c1
{
	uint8_t I2CBuff[2];
	int r, c, b, row, col;
	for (r = 0; r < 8; r++)
	{
		for (c = 0; c < 128; c++)
		{
			uint8_t temp = 0x00;
			for (b = 0; b < 8; b++)
			{
				row = r*8 + b;
				col = c;
				temp |= (frame[col][row]) << b;
			}
			//
			I2CBuff[0] = 0x40;	//to write data
			I2CBuff[1] = temp;
			HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 5);
		}
	}
}
void OLED::drawFullscreen(bool newFrame[128][64]) //I2C_HandleTypeDef *hi2c1
{

	for(int r = 0; r < 64; r++)
	{
		for(int c = 0; c < 128; c++)
		{
			//pixel(c,r,newFrame[c][r]);	//problem!
			frame[c][r] = newFrame[c][r];	//just used pixel() here for good measure
		}
	}

	drawFullscreen(); //hi2cI	//replaced the below!
}

void OLED::setContrast(uint8_t contrast)
{
	uint8_t I2CBuff[2];
	I2CBuff[0] = 0x00;	//to send command
	I2CBuff[1] = 0x81;
	HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 5);
	I2CBuff[0] = 0x00;
	I2CBuff[1] = contrast;
	HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 5);
}

void OLED::inverseDisplay(bool isInverse)
{
	uint8_t I2CBuff[2];
	I2CBuff[0] = 0x00;	//to send command
	if (isInverse)
	{
		I2CBuff[1] = 0xA6;
	}
	else
	{
		I2CBuff[1] = 0xA7;
	}
	HAL_I2C_Master_Transmit(hi2cI, CAddress, I2CBuff, 2, 5);
}

uint8_t OLED::getCAddress()
{
	return CAddress;
}

uint8_t OLED::getDAddress()
{
	return DAddress;
}

void OLED::pixel(int x, int y, bool colour, bool locked)	//use locked when inconvenient to lock from outside
{
	if(locked)
	{
		if(x < 0 || y < 0 || x >= width || y >= height)
		{
			return;
		}
	}
	frame[x][y] = colour;
}

void OLED::pixels3V(int x0, int y0, bool colour, bool locked)
{
	pixel(x0, y0, colour, locked);
	//
	pixel(x0, y0+1, colour, locked);
	pixel(x0, y0-1, colour, locked);
}
void OLED::pixels3H(int x0, int y0, bool colour, bool locked)
{
	pixel(x0, y0, colour, locked);
	//
	pixel(x0+1, y0, colour, locked);
	pixel(x0-1, y0, colour, locked);
}

void OLED::line(int x0, int y0, int x1, int y1, bool colour, bool locked)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0<=x1; x0++)
	{
		if (steep)
		{
			pixel(y0,x0,colour, locked);	//frame[y0][x0] = colour;
		}
		else
		{
			pixel(x0,y0,colour, locked);	//frame[x0][y0] = colour;
		}
		err -= dy;
		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}
void OLED::line(int x0, int y0, int x1, int y1, bool colour, int lineWidth)
{
	//draw the main line
	line(x0, y0, x1, y1, colour);

	//draw lines to its sides
	for (int i = 1; i < lineWidth; i++)
	{
		int x01 = 0, x02 = 0, x11 = 0, x12 = 0, y01 = 0, y02 = 0, y11 = 0, y12 = 0;
		if (x0 == x1)
		{
			x01 = x0 - i;
			x02 = x0 + i;
			y01 = y0;
			y02 = y0;
			x11 = x1 - i;
			x12 = x1 + i;
			y11 = y1;
			y12 = y1;
		}
		else if (y0 == y1)
		{
			x01 = x0;
			x02 = x0;
			y01 = y0 - i;
			y02 = y0 + i;
			x11 = x1;
			x12 = x1;
			y11 = y1 - i;
			y12 = y1 + i;
		}
		else
		{
			int I = i - 1;
			int o = I % 2;
			int n = (I - o) / 2;
			int a = n + 1;	//to multiply by x additive term in line 1, by y additive term in line 2
			int b = o + n;	//to multiply by y additive term in line 1, by x additive term in line 2
			//line 1
			if (x1 > x0)
			{
				x01 = x0 + a;
				y01 = y0 - b;
				x11 = x1 + a;
				y11 = y1 - b;
			}
			else if (x1 < x0)
			{
				x01 = x0 - a;
				y01 = y0 + b;
				x11 = x1 - a;
				y11 = y1 + b;
			}
			//line 2
			if (y1 > y0)
			{
				x02 = x0 - b;
				y02 = y0 + a;
				x12 = x1 - b;
				y12 = y1 + a;
			}
			else if (y1 < y0)
			{
				x02 = x0 + b;
				y02 = y0 - a;
				x12 = x1 + b;
				y12 = y1 - a;
			}
		}
		//draw the lines
		line(x01, y01, x11, y11, colour, true);
		line(x02, y02, x12, y12, colour, true);
	}
}
void OLED::fastHLine(int x0, int y0, int w, bool colour)
{
	line(x0, y0, x0+w-1, y0, colour);
}
void OLED::fastVLine(int x0, int y0, int h, bool colour)
{
	line(x0, y0, x0, y0+h-1, colour);
}

void OLED::triangle(int x0, int y0, int x1, int y1, int x2, int y2, bool colour)
{
	line(x0, y0, x1, y1, colour);
	line(x1, y1, x2, y2, colour);
	line(x2, y2, x0, y0, colour);
}
void OLED::triangle(int x0, int y0, int x1, int y1, int x2, int y2, bool colour, int lineWidth)
{
	line(x0, y0, x1, y1, colour, lineWidth);
	line(x1, y1, x2, y2, colour, lineWidth);
	line(x2, y2, x0, y0, colour, lineWidth);
}

void OLED::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, bool colour)
{
	int a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1)
	{
		swap(y0, y1); swap(x0, x1);
	}
	if (y1 > y2)
	{
		swap(y2, y1); swap(x2, x1);
	}
	if (y0 > y1)
	{
		swap(y0, y1); swap(x0, x1);
	}

	if (y0 == y2)
	{ // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if(x1 < a)      a = x1;
		else if(x1 > b) b = x1;
		if(x2 < a)      a = x2;
		else if(x2 > b) b = x2;
		fastHLine(a, y0, b-a+1, colour);
		return;
	}

	int16_t
	dx01 = x1 - x0,
	dy01 = y1 - y0,
	dx02 = x2 - x0,
	dy02 = y2 - y0,
	dx12 = x2 - x1,
	dy12 = y2 - y1,
	sa   = 0,
	sb   = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if(y1 == y2) last = y1;   // Include y1 scanline
	else         last = y1-1; // Skip it

	for(y=y0; y<=last; y++)
	{
		a   = x0 + sa / dy01;
		b   = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;

		if(a > b) swap(a,b);
		fastHLine(a, y, b-a+1, colour);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for(; y<=y2; y++)
	{
		a   = x1 + sa / dy12;
		b   = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;

		if(a > b) swap(a,b);
		fastHLine(a, y, b-a+1, colour);
	}
}

void OLED::circle(int x0, int y0, int r, bool colour)
{
	int f = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;

	pixel(x0, y0+r, colour);
	pixel(x0, y0-r, colour);
	pixel(x0+r, y0, colour);
	pixel(x0-r, y0, colour);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		pixel(x0 + x, y0 + y, colour);	//u
		pixel(x0 - x, y0 + y, colour);	//u
		pixel(x0 + x, y0 - y, colour);	//d
		pixel(x0 - x, y0 - y, colour);	//d
		pixel(x0 + y, y0 + x, colour);	//r
		pixel(x0 - y, y0 + x, colour);	//l
		pixel(x0 + y, y0 - x, colour);	//r
		pixel(x0 - y, y0 - x, colour);	//l
	}
}
void OLED::circle(int x0, int y0, int r, bool colour, int lineWidth)
{
	if(lineWidth == 1)
	{
		circle(x0, y0, r, colour);
	}
	else
	{
		//primary circle
		thiccCircle(x0, y0, r, colour);

		//intermediate circles around it
		for (int i = 1; i < lineWidth-1; i++)
		{
			thiccCircle(x0, y0, r+i, colour);
			thiccCircle(x0, y0, r-i, colour);
		}

		//external circles
		circle(x0, y0, r+(lineWidth-1), colour);
		circle(x0, y0, r-(lineWidth-1), colour);
	}
}
void OLED::thiccCircle(int x0, int y0, int r, bool colour)
{
	int f = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x = 0;
	int y = r;

/*
	pixel(x0, y0+r, colour);	//u
	pixel(x0, y0-r, colour);	//d
	pixel(x0+r, y0, colour);	//r
	pixel(x0-r, y0, colour);	//l
*/
	pixels3V(x0, y0+r, colour, true);	//u
	pixels3V(x0, y0-r, colour, true);	//d
	pixels3H(x0+r, y0, colour, true);	//r
	pixels3H(x0-r, y0, colour, true);	//l

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

/*
		pixel(x0 + x, y0 + y, colour);	//u
		pixel(x0 - x, y0 + y, colour);	//u
		pixel(x0 + x, y0 - y, colour);	//d
		pixel(x0 - x, y0 - y, colour);	//d
		pixel(x0 + y, y0 + x, colour);	//r
		pixel(x0 - y, y0 + x, colour);	//l
		pixel(x0 + y, y0 - x, colour);	//r
		pixel(x0 - y, y0 - x, colour);	//l
*/
		pixels3V(x0 + x, y0 + y, colour, true);	//u
		pixels3V(x0 - x, y0 + y, colour, true);	//u
		pixels3V(x0 + x, y0 - y, colour, true);	//d
		pixels3V(x0 - x, y0 - y, colour, true);	//d
		pixels3H(x0 + y, y0 + x, colour, true);	//r
		pixels3H(x0 - y, y0 + x, colour, true);	//l
		pixels3H(x0 + y, y0 - x, colour, true);	//r
		pixels3H(x0 - y, y0 - x, colour, true);	//l
	}
}

void OLED::fillCircle(int x0, int y0, int r, bool colour)
{
	fastVLine(x0, y0-r, 2*r+1, colour);
	fillCircleHelper(x0, y0, r, 3, 0, colour);
}
void OLED::fillCircleHelper(int x0, int y0, int r, int cornername, int delta, bool colour)	//was static
{
	int f     = 1 - r;
	int ddF_x = 1;
	int ddF_y = -2 * r;
	int x     = 0;
	int y     = r;

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x1)
		{
			fastVLine(x0+x, y0-y, 2*y+1+delta, colour);
			fastVLine(x0+y, y0-x, 2*x+1+delta, colour);
		}
		if (cornername & 0x2)
		{
		  fastVLine(x0-x, y0-y, 2*y+1+delta, colour);
		  fastVLine(x0-y, y0-x, 2*x+1+delta, colour);
		}
	}
}

void OLED::rectangle(int x0, int y0, int x1, int y1, bool colour)
{
	int rectWidth = x1 - x0;
	int rectHeight = y1 - y0;

	line(x0, y0, x1, y1-rectHeight, colour);
	line(x0, y0+rectHeight, x1, y1, colour);
	line(x0, y0, x0, y0+rectHeight, colour);
	line(x1, y1-rectHeight, x1, y1, colour);
}
void OLED::rectangle(int x0, int y0, int x1, int y1, bool colour, int lineWidth)
{
	int rectWidth = x1 - x0;
	int rectHeight = y1 - y0;

	line(x0, y0, x1, y1-rectHeight, colour, lineWidth);
	line(x0, y0+rectHeight, x1, y1, colour, lineWidth);
	line(x0, y0, x0, y0+rectHeight, colour, lineWidth);
	line(x1, y1-rectHeight, x1, y1, colour, lineWidth);
}

void OLED::fillRect(int x0, int y0, int x1, int y1, bool colour)
{
	fill(x0, y0, x1, y1, colour);
}

void OLED::connectPoints(int X[], int Y[], int pointsNum, bool colour)
{
	for(int i = 0; i < pointsNum - 1; i++)
	{
		line(X[i], Y[i], X[i+1], Y[i+1], colour);
	}
}
void OLED::connectPoints(int X[], int Y[], int pointsNum, bool colour, int lineWidth)
{
	for(int i = 0; i < pointsNum - 1; i++)
	{
		line(X[i], Y[i], X[i+1], Y[i+1], colour, lineWidth);
	}
}

void OLED::polygon(int X[], int Y[], int pointsNum, bool colour)
{
	connectPoints(X, Y, pointsNum, colour);
	line(X[pointsNum - 1], Y[pointsNum - 1], X[0], Y[0], colour);
}
void OLED::polygon(int X[], int Y[], int pointsNum, bool colour, int lineWidth)
{
	connectPoints(X, Y, pointsNum, colour, lineWidth);
	line(X[pointsNum - 1], Y[pointsNum - 1], X[0], Y[0], colour, lineWidth);
}

void OLED::fill(bool colour)
{
	for(int r = 0; r < 64; r++)
	{
		for(int c = 0; c < 128; c++)
		{
			pixel(c,r,colour);	//frame[c][r] = colour;
		}
	}
}
void OLED::fill(int x0, int y0, int x1, int y1, bool colour)
{
	for(int r = y0; r <= y1; r++)
	{
		for(int c = x0; c <= x1; c++)
		{
			pixel(c, r, colour, 1);	//frame[c][r] = colour;
		}
	}
}

void OLED::invert()
{
	for(int r = 0; r < 64; r++)
	{
		for(int c = 0; c < 128; c++)
		{
			pixel(c,r,!frame[c][r]);	//frame[c][r] = !frame[c][r];	//just used pixel() here for good measure
		}
	}
}
void OLED::invert(int x0, int y0, int x1, int y1)
{
	for(int r = y0; r <= y1; r++)
	{
		for(int c = x0; c <= x1; c++)
		{
			pixel(c,r,!frame[c][r]);	//frame[c][r] = !frame[c][r];
		}
	}
}

void OLED::character(int x, int y, unsigned char c, bool colour, bool bg, int size)
{
	if((x >= width)            || // Clip right
	 (y >= height)           || // Clip bottom
	 ((x + 6 * size - 1) < 0) || // Clip left
	 ((y + 8 * size - 1) < 0))   // Clip top
	return;

	if(!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

	for (int8_t i=0; i<6; i++ )
	{
		uint8_t line;
		if (i == 5)
			line = 0x0;
		else
		{
			line = pgm_read_byte(&font1[(c*5)+i]);
		}
		for (int8_t j = 0; j<8; j++)
		{
			if (line & 0x1)
			{
				if (size == 1) // default size
				{
					pixel(x+i, y+j, colour, 1);	//frame[x+i][y+j] = colour;
				}
				else
				{  // big size
					fillRect(x+(i*size), y+(j*size), size + x+(i*size), size+1 + y+(j*size), colour);
				}
			}
			else if (bg != colour)
			{
				if (size == 1) // default size
				{
					pixel(x+i, y+j, colour, 1);	//frame[x+i][y+j] = bg;
				}
				else
				{  // big size
					fillRect(x+i*size, y+j*size, size + x+i*size, size+1 + y+j*size, bg);
				}
			}
			line >>= 1;
		}
	}
}

void OLED::text(int x, int y, string s, bool colour, bool bg, int size)
{
	int offset = size*6;
	for(string::size_type i = 0; i < s.size(); i++)
	{
		character(x+(offset*i), y, s[i], colour, bg, size);
	}
}
