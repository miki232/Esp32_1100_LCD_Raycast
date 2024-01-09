/**
 * @file main.ino
 * @brief Raycasting implementation for rendering a 3D view on an LCD screen.
 *
 * This sketch uses raycasting technique to render a 3D view of a map on an LCD screen.
 * The map is represented as a 2D array of integers, where each integer represents a different type of wall or empty space.
 * The player's position, direction, and camera plane are used to cast rays and determine the distance to the walls.
 * The resulting distances are used to calculate the height of each vertical line in the 3D view.
 * The view is then rendered on the LCD screen using a buffer.
 *
 * The sketch also includes button handling for player movement and rotation.
 * The buttons are connected to the Arduino pins specified in the code.
 *
 * To use this sketch, connect the LCD pins to the Arduino pins as specified in the comments.
 * If using a 3.3V variant of Arduino, no additional components are required.
 * Otherwise, additional components may be needed to connect the 3.3V LCD to the digital pins of the Arduino.
 *
 * @note This sketch requires the PCF8814 library for controlling the LCD.
 *
 * The Raycasting algorithm is based on the tutorial by Lode Vandevenne:
 * http://lodev.org/cgtutor/raycasting.html 
 * LCD LIBRARY: https://github.com/vasiliucatalinmihai/Arduino-library-nokia-1100-lcd--PCF8814/tree/master
 */

/*
 * To use this sketch, connect the eight pins from your LCD like thus:
 * or you can change the pins in the #define section in the PCF8814.h.
 *
 *  LCD pin    Arduino pin
 * ---------------------
 * 0 N/C		N/C      /only on 1100 9 pin variant
 * 1 (LED+)	  +3.3V
 * 2 (VDD)    +3.3V
 * 3 (VDDI)   +3.3V
 * 4 (CLK)    25        // (SCLK)
 * 5 (SDIN)   26 		// (SDA)
 * 6 (GND)    GND		// (VSS)
 * 7 (SCE)    32 		// (XCS)
 * 8 (RST)    33 		// (XRES)
 * 
 
 *
 * Since these LCDs are +3.3V devices, you have to add extra components to
 * connect it to the digital pins of the Arduino (not necessary if you are
 * using a 3.3V variant of the Arduino, such as Sparkfun's Arduino Pro).
 */




#include <PCF8814.h>
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "sys/time.h"
#include <math.h>
#define SCREEN_HEIGHT 65
#define SCREEN_WIDTH 96
#define BT_RIGHT 14  // the number of the right pushbutton pin
#define BT_UP    12  // the number of the up pushbutton pin
#define BT_LEFT  27  // the number of the left pushbutton pin


#define mapWidth 24
#define mapHeight 24

static PCF8814 lcd;

int worldMap[mapWidth][mapHeight]=
{
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,2,2,2,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
	{1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,3,0,0,0,3,0,0,0,1},
	{1,0,0,0,0,0,2,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,2,2,0,2,2,0,0,0,0,3,0,3,0,3,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,0,0,0,5,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
	{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

double posX = 6, posY = 4;  //x and y start position
double dirX = -1, dirY = 0; //initial direction vector
double planeX = 0, planeY = 0.66; //the 2d raycaster version of camera plane

byte buffer[SCREEN_WIDTH * SCREEN_HEIGHT];

bool  drawed = false;

void writeToBuffer(int bufferWidth, int x, int y, int color)
{
	//Calcola la posizione nella mappa di bit
	int mapBit = y % 8;
	int mapByte = y / 8;

	// Calcola l'offset nel buffer
	int bufferOffset = x + mapByte * bufferWidth;

	// Setta il bit appropriato nel byte corrente nel buffer
	if (color == 1 && drawed == false)
	{
		buffer[bufferOffset] |= (1 << mapBit);
		drawed = true;
	}
	else if (color == 1 && drawed)
	{
		buffer[bufferOffset] &= ~(1 << mapBit);
		drawed = false;
	}
	else
		buffer[bufferOffset] |= (1 << mapBit);
}

// Clear the buffer
void clearBuffer(byte* buffer, int bufferSize)
{
	memset(buffer, 0, bufferSize);
}

// variable for reading the pushbutton status
int left = 0;  
int right = 0;
int up = 0;

void setup()
{
	pinMode(BT_LEFT, INPUT_PULLUP);
	pinMode(BT_RIGHT, INPUT_PULLUP);
	pinMode(BT_UP, INPUT_PULLUP);

	lcd.begin();
	delay(2000);
	lcd.clear();
}

void loop()
{
	clearBuffer(buffer, sizeof(buffer));
	for(int x = 0; x < SCREEN_WIDTH; x++)
	{
		double cameraX = 2 * x / (double)SCREEN_WIDTH - 1;
		double rayDirX = dirX + planeX * cameraX;
		double rayDirY = dirY + planeY * cameraX;
		int mapX = int(posX);
		int mapY = int(posY);

		double sideDistX;
		double sideDistY;
		double deltaDistX = (rayDirX == 0) ? 1e30 : 1 / std::abs(rayDirX);
		double deltaDistY = (rayDirY == 0) ? 1e30 : 1 / std::abs(rayDirY);

		double perpWallDist;

		int stepX;
		int stepY;

		int hit = 0;
		int side;
		if(rayDirX < 0)
		{
			stepX = -1;
			sideDistX = (posX - mapX) * deltaDistX;
		}
		else
		{
			stepX = 1;
			sideDistX = (mapX + 1.0 - posX) * deltaDistX;
		}
		if(rayDirY < 0)
		{
			stepY = -1;
			sideDistY = (posY - mapY) * deltaDistY;
		}
		else
		{
			stepY = 1;
			sideDistY = (mapY + 1.0 - posY) * deltaDistY;
		}
		while(hit == 0)
		{
			if(sideDistX < sideDistY)
			{
				sideDistX += deltaDistX;
				mapX += stepX;
				side = 0;
			}
			else
			{
				sideDistY += deltaDistY;
				mapY += stepY;
				side = 1;
			}
			if(worldMap[mapX][mapY] > 0) hit = 1;
		}
		if(side == 0)
			perpWallDist = (sideDistX - deltaDistX);
		else
			perpWallDist = (sideDistY - deltaDistY);

		int lineHeight = (int)(SCREEN_HEIGHT / perpWallDist);
		int drawStart = -lineHeight / 2 + SCREEN_HEIGHT/ 2;
		if(drawStart < 0) drawStart = 0;

		int drawEnd = lineHeight / 2 + SCREEN_HEIGHT/ 2;
		if(drawEnd >= SCREEN_HEIGHT) drawEnd = SCREEN_HEIGHT- 1;

		int y = drawStart;
		while (y <= drawEnd)
		{
			writeToBuffer(96, x, y, side); // 96 is the buffer width
			y++ ;
		}
	}
	lcd.drawBitmap(buffer, SCREEN_WIDTH, SCREEN_HEIGHT);

	//Key management
	double moveSpeed = 0.03;
	double rotSpeed = 0.03;
	left = digitalRead(BT_LEFT);
	right = digitalRead(BT_RIGHT);
	up = digitalRead(BT_UP);
	if(up == !HIGH)
	{
		if(worldMap[int(posX + dirX * moveSpeed)][int(posY)] == false) posX += dirX * moveSpeed;
		if(worldMap[int(posX)][int(posY + dirY * moveSpeed)] == false) posY += dirY * moveSpeed;
	}
	if(right == !HIGH)
	{
		double oldDirX = dirX;
		dirX = dirX * cos(-rotSpeed) - dirY * sin(-rotSpeed);
		dirY = oldDirX * sin(-rotSpeed) + dirY * cos(-rotSpeed);
		double oldPlaneX = planeX;
		planeX = planeX * cos(-rotSpeed) - planeY * sin(-rotSpeed);
		planeY = oldPlaneX * sin(-rotSpeed) + planeY * cos(-rotSpeed);
	}
	if(left == !HIGH)
	{
		double oldDirX = dirX;
		dirX = dirX * cos(rotSpeed) - dirY * sin(rotSpeed);
		dirY = oldDirX * sin(rotSpeed) + dirY * cos(rotSpeed);
		double oldPlaneX = planeX;
		planeX = planeX * cos(rotSpeed) - planeY * sin(rotSpeed);
		planeY = oldPlaneX * sin(rotSpeed) + planeY * cos(rotSpeed);
	}
}