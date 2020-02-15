/*----------------------------------------------------------------------------------
    Description:    Main program for the Audio Spectrum Analyzer v2 based on a 
                    RaspberryPi 3.
    Date:           11/14/2018
    Author:         Phillip Durdaut
----------------------------------------------------------------------------------*/

#include "main.h"

/*----------------------------------------------------------------------------------
  Global variables
----------------------------------------------------------------------------------*/
pthread_t tRecord;
pthread_t tProcess;
pthread_t tDisplay;

u8_t flagAveragedMagnitudeSpectrumBlockedForWriting = 0;

snd_pcm_t *captureHandle;
snd_pcm_hw_params_t *hwParams;

u32_t frameSize = FRAME_SIZE;
u32_t sampleRate = SAMPLE_RATE;

s16_t* inputBuffer1;
s16_t* inputBuffer2;
u8_t bufferToRecordTo = 1;
u8_t bufferToWorkWith= 2;

float hanningWindow[FRAME_SIZE];
fftw_complex *fftInputBuffer;
fftw_complex *fftOutputBuffer;
fftw_plan fftPlan;

u8_t numberOfFreqBands = FREQUENCY_BANDS;
float freqBands[] = 	 { 10, 25, 31.5, 40, 50, 63, 80, 100, 125, 180, 200, 250, 315, 400, 500, 630, 800, 1e3, 1.25e3, 1.6e3, 2e3, 2.5e3, 3.15e3, 4e3, 5e3, 6.3e3, 8e3, 10e3, 12.5e3, 16e3, 20e3 };
float corrFactors_dB[] = { 0,  -1, 0,    -2,  0, -2, -1, -1,  0,   -1,  2,   4,   4,   4,   7,   9,   11,  12,  14,     16,    17,  20,    21,     23,  25,  27,    29,  32,   34,     36,   40 };
float corrOffset_dB = -130;
float normalizedBandwidth = 0.25;

u8_t numberOfAverages = AVERAGES;
u8_t averageCounter = 0;
float singleCompleteMagnitudeSpectrum[FRAME_SIZE];
float averageDataMagnitudeSpectra[AVERAGES][FREQUENCY_BANDS];
float averagedMagnitudeSpectrum[FREQUENCY_BANDS];
float averagedMagnitudeSpectrum_dB[FREQUENCY_BANDS];

u8_t numberOfLongAverages = LONGAVERAGES;
u8_t longAverageCounter = 0;
float longAverageDataMagnitudeSpectra[LONGAVERAGES][FREQUENCY_BANDS];
float longAveragedMagnitudeSpectrum[FREQUENCY_BANDS];
float longAveragedMagnitudeSpectrum_dB[FREQUENCY_BANDS];

u8_t ledMatrixMoinColumn00[NROWS] = LEDMATRIX_MOIN_ROW00;
u8_t ledMatrixMoinColumn01[NROWS] = LEDMATRIX_MOIN_ROW01;
u8_t ledMatrixMoinColumn02[NROWS] = LEDMATRIX_MOIN_ROW02;
u8_t ledMatrixMoinColumn03[NROWS] = LEDMATRIX_MOIN_ROW03;
u8_t ledMatrixMoinColumn04[NROWS] = LEDMATRIX_MOIN_ROW04;
u8_t ledMatrixMoinColumn05[NROWS] = LEDMATRIX_MOIN_ROW05;
u8_t ledMatrixMoinColumn06[NROWS] = LEDMATRIX_MOIN_ROW06;
u8_t ledMatrixMoinColumn07[NROWS] = LEDMATRIX_MOIN_ROW07;
u8_t ledMatrixMoinColumn08[NROWS] = LEDMATRIX_MOIN_ROW08;
u8_t ledMatrixMoinColumn09[NROWS] = LEDMATRIX_MOIN_ROW09;
u8_t ledMatrixMoinColumn10[NROWS] = LEDMATRIX_MOIN_ROW10;
u8_t ledMatrixMoinColumn11[NROWS] = LEDMATRIX_MOIN_ROW11;
u8_t ledMatrixMoinColumn12[NROWS] = LEDMATRIX_MOIN_ROW12;
u8_t ledMatrixMoinColumn13[NROWS] = LEDMATRIX_MOIN_ROW13;
u8_t ledMatrixMoinColumn14[NROWS] = LEDMATRIX_MOIN_ROW14;
u8_t ledMatrixMoinColumn15[NROWS] = LEDMATRIX_MOIN_ROW15;
u8_t ledMatrixMoinColumn16[NROWS] = LEDMATRIX_MOIN_ROW16;
u8_t ledMatrixMoinColumn17[NROWS] = LEDMATRIX_MOIN_ROW17;
u8_t ledMatrixMoinColumn18[NROWS] = LEDMATRIX_MOIN_ROW18;
u8_t ledMatrixMoinColumn19[NROWS] = LEDMATRIX_MOIN_ROW19;
u8_t ledMatrixMoinColumn20[NROWS] = LEDMATRIX_MOIN_ROW20;
u8_t ledMatrixMoinColumn21[NROWS] = LEDMATRIX_MOIN_ROW21;
u8_t ledMatrixMoinColumn22[NROWS] = LEDMATRIX_MOIN_ROW22;
u8_t ledMatrixMoinColumn23[NROWS] = LEDMATRIX_MOIN_ROW23;
u8_t ledMatrixMoinColumn24[NROWS] = LEDMATRIX_MOIN_ROW24;
u8_t ledMatrixMoinColumn25[NROWS] = LEDMATRIX_MOIN_ROW25;
u8_t ledMatrixMoinColumn26[NROWS] = LEDMATRIX_MOIN_ROW26;
u8_t ledMatrixMoinColumn27[NROWS] = LEDMATRIX_MOIN_ROW27;
u8_t ledMatrixMoinColumn28[NROWS] = LEDMATRIX_MOIN_ROW28;
u8_t ledMatrixMoinColumn29[NROWS] = LEDMATRIX_MOIN_ROW29;
u8_t ledMatrixMoinColumn30[NROWS] = LEDMATRIX_MOIN_ROW30;
u8_t ledMatrixMoinColumn31[NROWS] = LEDMATRIX_MOIN_ROW31;

/*----------------------------------------------------------------------------------
  Prototypes of the threads
----------------------------------------------------------------------------------*/
void *threadRecord();
void *threadProcess();
void *threadDisplay();

/*----------------------------------------------------------------------------------
  Prototypes of the private functions
----------------------------------------------------------------------------------*/
static void gpioInit();
static void audioInit();
static void hanningWindowInit();
static void fftInit();
static void audioRecord(s16_t* p_buffer);
static u16_t freqToIndex(float p_freq);
static void enableRow(u8_t p_rowToEnable);
static void disableRow(u8_t p_rowToDisable);
static void disableAllColumns();
static void enableColumn(u8_t p_columnToEnable);

/*----------------------------------------------------------------------------------
  Main function
----------------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    /* initialization */    
    gpioInit();
    audioInit();
    hanningWindowInit();
    fftInit();
    
    /* memory allocation */
    inputBuffer1 = malloc(FRAME_SIZE * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE)/8);
    inputBuffer2 = malloc(FRAME_SIZE * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE)/8);
    
    /* start threads for recording and displaying */
    pthread_create(&tRecord, NULL, threadRecord, NULL);
    pthread_create(&tDisplay, NULL, threadDisplay, NULL);	
    
    /* wait forever */
    pthread_join(tRecord, NULL);
    pthread_join(tDisplay, NULL);
	
	/* never reached */
    return 0;
}

void *threadRecord()
{
	s16_t* inputBuffer;
	
	while (1)
	{
		/* decide which buffer to work with */
		if (bufferToRecordTo == 1)
		{
			inputBuffer = inputBuffer1;
			bufferToRecordTo = 2;
			bufferToWorkWith = 1;	
		}
		else if (bufferToRecordTo == 2)
		{
			inputBuffer = inputBuffer2;
			bufferToRecordTo = 1;
			bufferToWorkWith = 2;
		}
		
		/* record audio */
		audioRecord(inputBuffer);
		
		/* start the processing thread */
		pthread_join(tProcess, NULL);
		pthread_create(&tProcess, NULL, threadProcess, NULL);
	}
} 

void *threadProcess()
{
	s16_t* inputBuffer;
	
	/* decide which buffer to work with */
	if (bufferToWorkWith == 1)
	{
		inputBuffer = inputBuffer1;
	}
	else if (bufferToWorkWith == 2)
	{
		inputBuffer = inputBuffer2;
	}
	
	/* perform fft on a copy of the input buffer which is weighted by a window */	
	for (u16_t i = 0; i < FRAME_SIZE; i++)
    {
		fftInputBuffer[i][0] = (double)(((float)(inputBuffer[i])) * hanningWindow[i]);
		fftInputBuffer[i][1] = (double)(0.0f);
	}
	fftw_execute(fftPlan);	
	
	/* calculate the magnitude spectrum */
	for (u16_t i = 0; i < FRAME_SIZE; i++)
    {
		singleCompleteMagnitudeSpectrum[i] = (float)sqrt(pow(fftOutputBuffer[i][0], 2.0f) + pow(fftOutputBuffer[i][1], 2.0f));
	}
	
	/* calculate current magnitude spectrum */
	for (u16_t i = 0; i < numberOfFreqBands; i++)
	{
		float centerFreq = freqBands[i];
		float lowerFreq = centerFreq * (1-normalizedBandwidth);
		float upperFreq = centerFreq * (1+normalizedBandwidth);

		int lowerFreqIndex = freqToIndex(lowerFreq);
		int upperFreqIndex = freqToIndex(upperFreq);

		float tmp = 0;
		for (u16_t j = lowerFreqIndex; j <= upperFreqIndex; j++)
		{
			tmp = tmp + singleCompleteMagnitudeSpectrum[j];
		}
		tmp = tmp / (upperFreqIndex-lowerFreqIndex+1);

		averageDataMagnitudeSpectra[averageCounter][i] = tmp;
		averageCounter++;
		if (averageCounter == numberOfAverages)
			averageCounter = 0;
			
		longAverageDataMagnitudeSpectra[longAverageCounter][i] = tmp;
		longAverageCounter++;
		if (longAverageCounter == numberOfLongAverages)
			longAverageCounter = 0;
	}
	
	/* calculate averaged spectrum */
	flagAveragedMagnitudeSpectrumBlockedForWriting = 1;
	
	for (u16_t i = 0; i < numberOfFreqBands; i++)
	{  
		averagedMagnitudeSpectrum[i] = 0;
		for (u16_t j = 0; j < numberOfAverages; j++)
		{
			averagedMagnitudeSpectrum[i] = averagedMagnitudeSpectrum[i] + averageDataMagnitudeSpectra[j][i];
		}
		averagedMagnitudeSpectrum[i] = (float)(averagedMagnitudeSpectrum[i]/((float)(numberOfAverages)));
		averagedMagnitudeSpectrum_dB[i] = (float)(20*log10(averagedMagnitudeSpectrum[i]) + corrFactors_dB[i] + corrOffset_dB);
	}
	
	for (u16_t i = 0; i < numberOfFreqBands; i++)
	{  
		longAveragedMagnitudeSpectrum[i] = 0;
		for (u16_t j = 0; j < numberOfLongAverages; j++)
		{
			if (longAverageDataMagnitudeSpectra[j][i] > longAveragedMagnitudeSpectrum[i])
				longAveragedMagnitudeSpectrum[i] = longAverageDataMagnitudeSpectra[j][i];
		}
		//longAveragedMagnitudeSpectrum[i] = (float)(longAveragedMagnitudeSpectrum[i]/((float)(numberOfLongAverages)));
		longAveragedMagnitudeSpectrum_dB[i] = (float)(20*log10(longAveragedMagnitudeSpectrum[i]) + corrFactors_dB[i] + corrOffset_dB);
	}
	
	flagAveragedMagnitudeSpectrumBlockedForWriting = 0;
	
	pthread_exit(0);
} 

void *threadDisplay()
{
	u8_t typePosNegMuin = 0;
	u32_t counterPosNegMuin = 0;
	u8_t blinkCounter = 0;
	u8_t activatedCol = 0;
	while(1)
	{
		activatedCol++;	
		if (activatedCol == 33)
			activatedCol = 1;		
		enableColumn(activatedCol);
		
		u8_t* pCurrentColumn;
		switch(activatedCol)
		{
			case 1: pCurrentColumn = ledMatrixMoinColumn00;	break;
			case 2: pCurrentColumn = ledMatrixMoinColumn01;	break;
			case 3: pCurrentColumn = ledMatrixMoinColumn02;	break;
			case 4: pCurrentColumn = ledMatrixMoinColumn03;	break;
			case 5: pCurrentColumn = ledMatrixMoinColumn04;	break;
			case 6: pCurrentColumn = ledMatrixMoinColumn05;	break;
			case 7: pCurrentColumn = ledMatrixMoinColumn06;	break;
			case 8: pCurrentColumn = ledMatrixMoinColumn07;	break;
			case 9: pCurrentColumn = ledMatrixMoinColumn08;	break;
			case 10: pCurrentColumn = ledMatrixMoinColumn09; break;
			case 11: pCurrentColumn = ledMatrixMoinColumn10; break;
			case 12: pCurrentColumn = ledMatrixMoinColumn11; break;
			case 13: pCurrentColumn = ledMatrixMoinColumn12; break;
			case 14: pCurrentColumn = ledMatrixMoinColumn13; break;
			case 15: pCurrentColumn = ledMatrixMoinColumn14; break;
			case 16: pCurrentColumn = ledMatrixMoinColumn15; break;
			case 17: pCurrentColumn = ledMatrixMoinColumn16; break;
			case 18: pCurrentColumn = ledMatrixMoinColumn17; break;
			case 19: pCurrentColumn = ledMatrixMoinColumn18; break;
			case 20: pCurrentColumn = ledMatrixMoinColumn19; break;
			case 21: pCurrentColumn = ledMatrixMoinColumn20; break;
			case 22: pCurrentColumn = ledMatrixMoinColumn21; break;
			case 23: pCurrentColumn = ledMatrixMoinColumn22; break;
			case 24: pCurrentColumn = ledMatrixMoinColumn23; break;
			case 25: pCurrentColumn = ledMatrixMoinColumn24; break;
			case 26: pCurrentColumn = ledMatrixMoinColumn25; break;
			case 27: pCurrentColumn = ledMatrixMoinColumn26; break;
			case 28: pCurrentColumn = ledMatrixMoinColumn27; break;
			case 29: pCurrentColumn = ledMatrixMoinColumn28; break;
			case 30: pCurrentColumn = ledMatrixMoinColumn29; break;
			case 31: pCurrentColumn = ledMatrixMoinColumn30; break;
			case 32: pCurrentColumn = ledMatrixMoinColumn31; break;
		}		
		
		for (u8_t n = 0; n < NROWS; n++)
		{
			if (typePosNegMuin)
			{
				if (pCurrentColumn[n] == 1)
					enableRow(NROWS-n);
				else
					disableRow(NROWS-n);
			}
			else
			{
				if (pCurrentColumn[n] == 1)
					disableRow(NROWS-n);
				else
					enableRow(NROWS-n);
			}
		}
		
		counterPosNegMuin++;
		if (counterPosNegMuin == 1000)
		{
			typePosNegMuin = !typePosNegMuin;
			counterPosNegMuin = 0;
			blinkCounter++;
			
			if (blinkCounter == 25)
			break;
		}
		
		delay_us(100);
		disableAllColumns();
	}
	
	while(1)
	{
		while(flagAveragedMagnitudeSpectrumBlockedForWriting == 1);
			delay_us(10);
		
		for (u8_t i = 0; i < numberOfFreqBands; i++)
		{	
			disableAllColumns();
			
			for (u8_t n = 0; n < NROWS; n++)
			{
				if (averagedMagnitudeSpectrum_dB[i] >= ((3*STEP_DB)-(STEP_DB*n)))
					enableRow(n+1);
				else
					disableRow(n+1);
			}
			
			if (digitalRead(PIN_SW) == HIGH)
			{
				for (u8_t n = 0; n < NROWS; n++)
				{
					if (longAveragedMagnitudeSpectrum_dB[i] >= (9-(3*n)))
					{
						enableRow(n+1);
						break;
					}
				}	
			}
			
			enableColumn(i+1);
			delay_us(100);
		}
	}
}

/*----------------------------------------------------------------------------------
  Private functions
----------------------------------------------------------------------------------*/

static void gpioInit()
{
	// Invoke library	
	wiringPiSetup();
	
	// Mode
	pinMode(PIN_R1, OUTPUT);
	pinMode(PIN_R2, OUTPUT);
	pinMode(PIN_R3, OUTPUT);
	pinMode(PIN_R4, OUTPUT);
	pinMode(PIN_R5, OUTPUT);
	pinMode(PIN_R6, OUTPUT);
	pinMode(PIN_R7, OUTPUT);
	pinMode(PIN_R8, OUTPUT);
	pinMode(PIN_R9, OUTPUT);
	pinMode(PIN_R10, OUTPUT);
	pinMode(PIN_R11, OUTPUT);
	pinMode(PIN_R12, OUTPUT); 

	pinMode(PIN_D1nEN, OUTPUT);
	pinMode(PIN_D1A, OUTPUT);
	pinMode(PIN_D1B, OUTPUT);
	pinMode(PIN_D1C, OUTPUT);
	pinMode(PIN_D1D, OUTPUT);

	pinMode(PIN_D2nEN, OUTPUT);
	pinMode(PIN_D2A, OUTPUT);
	pinMode(PIN_D2B, OUTPUT);
	pinMode(PIN_D2C, OUTPUT);
	pinMode(PIN_D2D, OUTPUT);
	
	pinMode(PIN_SW, INPUT);
	
	// Value
	digitalWrite(PIN_R1, LOW);
	digitalWrite(PIN_R2, LOW);
	digitalWrite(PIN_R3, LOW);
	digitalWrite(PIN_R4, LOW);
	digitalWrite(PIN_R5, LOW);
	digitalWrite(PIN_R6, LOW);
	digitalWrite(PIN_R7, LOW);
	digitalWrite(PIN_R8, LOW);
	digitalWrite(PIN_R9, LOW);
	digitalWrite(PIN_R10, LOW);
	digitalWrite(PIN_R11, LOW);
	digitalWrite(PIN_R12, LOW);

	digitalWrite(PIN_D1nEN, HIGH);
	digitalWrite(PIN_D1A, LOW);
	digitalWrite(PIN_D1B, LOW);
	digitalWrite(PIN_D1C, LOW);
	digitalWrite(PIN_D1D, LOW);

	digitalWrite(PIN_D2nEN, HIGH);
	digitalWrite(PIN_D2A, LOW);
	digitalWrite(PIN_D2B, LOW);
	digitalWrite(PIN_D2C, LOW);
	digitalWrite(PIN_D2D, LOW);
}

static void audioInit()
{
	int err;
	
	if ((err = snd_pcm_open (&captureHandle, "hw:1,0", SND_PCM_STREAM_CAPTURE, 0)) < 0)
	{
		fprintf(stderr, "Cannot open audio device hw:1,0 (%s).\n", snd_strerror(err));
		exit(1);
	}

	if ((err = snd_pcm_hw_params_malloc(&hwParams)) < 0)
	{
		fprintf(stderr, "Cannot allocate hardware parameter structure (%s).\n", snd_strerror(err));
		exit(1);
	}

	if ((err = snd_pcm_hw_params_any (captureHandle, hwParams)) < 0)
	{
		fprintf(stderr, "Cannot initialize hardware parameter structure (%s).\n", snd_strerror (err));
		exit(1);
	}

	if ((err = snd_pcm_hw_params_set_access(captureHandle, hwParams, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
	{
		fprintf(stderr, "Cannot set access type (%s).\n", snd_strerror(err));
		exit(1);
	}

	if ((err = snd_pcm_hw_params_set_format(captureHandle, hwParams, SND_PCM_FORMAT_S16_LE)) < 0)
	{
		fprintf(stderr, "Cannot set sample format (%s).\n",	snd_strerror(err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_rate_near(captureHandle, hwParams, &sampleRate, 0)) < 0)
	{
		fprintf(stderr, "Cannot set sample rate (%s).\n", snd_strerror(err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_channels(captureHandle, hwParams, 1)) < 0)
	{
		fprintf(stderr, "Cannot set channel count (%s).\n", snd_strerror(err));
		exit(1);
	}

	if ((err = snd_pcm_hw_params(captureHandle, hwParams)) < 0)
	{
		fprintf(stderr, "Cannot set parameters (%s).\n", snd_strerror(err));
		exit(1);
	}

	if ((err = snd_pcm_prepare(captureHandle)) < 0)
	{
		fprintf (stderr, "Cannot prepare audio interface for use (%s).\n", snd_strerror(err));
		exit(1);
	}
	
	snd_pcm_hw_params_free(hwParams);
}

static void hanningWindowInit()
{
    /*
     * Hanning window:
     *
     * w[n] = 0.5 - 0.5 * cos(2 * pi * n / N)
     *      = 0.5 * ( 1 - cos(2 * pi * n / N) )
     *
     * n = -N/2 .. N/2 - 1
     * N = FRAME_SIZE
     */
    for (u16_t n = 0; n < FRAME_SIZE; n++) {
        hanningWindow[n] = (float)(((0.5f * (1 - (cos((double)(2 * M_PI * (double)n / (double)FRAME_SIZE)))))));
    }
}

static void fftInit()
{
    fftInputBuffer = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * FRAME_SIZE);
    fftOutputBuffer = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * FRAME_SIZE);
    fftPlan = fftw_plan_dft_1d(FRAME_SIZE, fftInputBuffer, fftOutputBuffer, FFTW_FORWARD, FFTW_MEASURE);
}

static void audioRecord(s16_t* p_buffer)
{
	int err;
	
	if ((err = snd_pcm_readi(captureHandle, p_buffer, FRAME_SIZE)) != FRAME_SIZE)
	{
		fprintf(stderr, "Read from audio interface failed (%s).\n", snd_strerror(err));
		exit(1);
    }
}

static u16_t freqToIndex(float p_freq)
{
	float deltaFreq = ((float)sampleRate)/((float)frameSize);
	return round(p_freq/deltaFreq);
}

static void disableRow(u8_t p_rowToDisable)
{
  switch (p_rowToDisable)
  {
    case 1:  digitalWrite(PIN_R1, LOW);  break;
    case 2:  digitalWrite(PIN_R2, LOW);  break;
    case 3:  digitalWrite(PIN_R3, LOW);  break;
    case 4:  digitalWrite(PIN_R4, LOW);  break;
    case 5:  digitalWrite(PIN_R5, LOW);  break;
    case 6:  digitalWrite(PIN_R6, LOW);  break;
    case 7:  digitalWrite(PIN_R7, LOW);  break;
    case 8:  digitalWrite(PIN_R8, LOW);  break;
    case 9:  digitalWrite(PIN_R9, LOW);  break;
    case 10: digitalWrite(PIN_R10, LOW); break;
    case 11: digitalWrite(PIN_R11, LOW); break;
    case 12: digitalWrite(PIN_R12, LOW); break;
    default: break;
  }
}

static void enableRow(u8_t p_rowToEnable)
{
  switch (p_rowToEnable)
  {
    case 1:  digitalWrite(PIN_R1, HIGH);  break;
    case 2:  digitalWrite(PIN_R2, HIGH);  break;
    case 3:  digitalWrite(PIN_R3, HIGH);  break;
    case 4:  digitalWrite(PIN_R4, HIGH);  break;
    case 5:  digitalWrite(PIN_R5, HIGH);  break;
    case 6:  digitalWrite(PIN_R6, HIGH);  break;
    case 7:  digitalWrite(PIN_R7, HIGH);  break;
    case 8:  digitalWrite(PIN_R8, HIGH);  break;
    case 9:  digitalWrite(PIN_R9, HIGH);  break;
    case 10: digitalWrite(PIN_R10, HIGH); break;
    case 11: digitalWrite(PIN_R11, HIGH); break;
    case 12: digitalWrite(PIN_R12, HIGH); break;
    default: break;
  }
}

static void disableAllColumns()
{
  digitalWrite(PIN_D1nEN, HIGH);
  digitalWrite(PIN_D2nEN, HIGH);
}

static void enableColumn(u8_t p_columnToEnable)
{
  disableAllColumns();
    
  if (p_columnToEnable >= 1 && p_columnToEnable <= 16)
  {
    switch (p_columnToEnable)
    {
      case 1:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 2:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 3:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 4:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 5:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 6:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 7:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 8:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, LOW);
        break;
      case 9:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 10:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 11:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 12:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, LOW);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 13:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 14:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, LOW);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 15:
        digitalWrite(PIN_D1A, LOW);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, HIGH);
        break;
      case 16:
        digitalWrite(PIN_D1A, HIGH);
        digitalWrite(PIN_D1B, HIGH);
        digitalWrite(PIN_D1C, HIGH);
        digitalWrite(PIN_D1D, HIGH);
        break;
    } 
    
    digitalWrite(PIN_D1nEN, LOW);
  }
  else if (p_columnToEnable >= 17 && p_columnToEnable <= 32)
  {
    p_columnToEnable = p_columnToEnable - 16;    
    switch (p_columnToEnable)
    {
      case 1:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 2:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 3:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 4:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 5:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 6:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 7:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 8:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, LOW);
        break;
      case 9:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 10:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 11:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 12:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, LOW);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 13:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 14:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, LOW);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 15:
        digitalWrite(PIN_D2A, LOW);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, HIGH);
        break;
      case 16:
        digitalWrite(PIN_D2A, HIGH);
        digitalWrite(PIN_D2B, HIGH);
        digitalWrite(PIN_D2C, HIGH);
        digitalWrite(PIN_D2D, HIGH);
        break;
    }
    
    digitalWrite(PIN_D2nEN, LOW);
  }
}
