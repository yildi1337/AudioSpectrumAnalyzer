/*----------------------------------------------------------------------------------
    Description:    Generic includes and defines.
    Date:           05/26/2014
    Author:         Phillip Durdaut
----------------------------------------------------------------------------------*/

#ifndef MAIN_H_
#define MAIN_H_

/*----------------------------------------------------------------------------------
  Includes
----------------------------------------------------------------------------------*/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <time.h>
#include <alsa/asoundlib.h>
#include <pthread.h>
#include <fftw3.h>
#include <math.h>

#include "types.h"
#include "delay.h"

/*----------------------------------------------------------------------------------
  Defines
----------------------------------------------------------------------------------*/

#define FRAME_SIZE 		1024
#define SAMPLE_RATE 	44100
#define FREQUENCY_BANDS 31
#define AVERAGES 		2
#define LONGAVERAGES 	50
#define STEP_DB			3

#define NROWS			12
#define NCOLS			32

#define PIN_R1			13		//9
#define PIN_R2			6		//25
#define PIN_R3			14		//11
#define PIN_R4			10		//8
#define PIN_R5			11		//7
#define PIN_R6			21		//5
#define PIN_R7			22		//6
#define PIN_R8			26		//12
#define PIN_R9			23		//13
#define PIN_R10			24		//19
#define PIN_R11			27		//16
#define PIN_R12			28		//20

#define PIN_D1nEN		9		//3
#define PIN_D1A			3		//22
#define PIN_D1B			4		//23
#define PIN_D1C			5		//24
#define PIN_D1D			12		//10

#define PIN_D2nEN		8		//2
#define PIN_D2A			2		//27
#define PIN_D2B			7		//4
#define PIN_D2C			0		//17
#define PIN_D2D			1		//18

#define PIN_SW			25		//26

#define LEDMATRIX_MUIN_ROW00    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW01    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW02    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW03    { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 }
#define LEDMATRIX_MUIN_ROW04    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW05    { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW06    { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW07    { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW08    { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW09    { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 }
#define LEDMATRIX_MUIN_ROW10    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW11    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW12    { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW13    { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW14    { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW15    { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW16    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW17    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW18    { 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW19    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW20    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW21    { 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW22    { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW23    { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW24    { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW25    { 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW26    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW27    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW28    { 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0 }
#define LEDMATRIX_MUIN_ROW29    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW30    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define LEDMATRIX_MUIN_ROW31    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

#endif /* MAIN_H_ */
