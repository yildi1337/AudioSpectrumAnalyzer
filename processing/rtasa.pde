// Audio Spectrum Analyzer
// Phillip Durdaut, 2018

import ddf.minim.*;
import ddf.minim.analysis.*;

// Parameters
int bufferSize = 2048;
float sampleRate = 44100;
int bitDepth = 16;
float[] freqBands = { 10, 25, 31.5, 40, 50, 63, 80, 100, 125, 180, 200, 250, 315, 400, 500, 630, 800, 1e3, 1.25e3, 1.6e3, 2e3, 2.5e3, 3.15e3, 4e3, 5e3, 6.3e3, 8e3, 10e3, 12.5e3, 16e3, 20e3 };
int numberOfFreqBands = 31;
float normalizedBandwidth = 0.1;
int averages = 4;
int averagesRMS = 100;

// Helper variables
int averageCounter = 0;
float[][] averageData;
float[] averagedData;
float[] averagedData_dB;

int averageCounterRMS = 0;
float[] averageDataRMS;
float averagedDataRMS = 0;
float averagedDataRMS_dB = 0;

// Objects
Minim minim;
AudioInput audioInput;
FFT fft;

void setup()
{
  size(1000, 200, P3D);
  
  averageData = new float[averages][numberOfFreqBands];
  averagedData = new float[numberOfFreqBands];
  averagedData_dB = new float[numberOfFreqBands];
  
  averageDataRMS = new float[averagesRMS];
  
  minim = new Minim(this);
  audioInput = minim.getLineIn(Minim.STEREO, bufferSize, sampleRate, bitDepth);
  fft = new FFT(audioInput.bufferSize(), sampleRate);
}


void draw()
{  
  calculateCurrentSpectrumAndRMS();
  calculateAverages();
  updateDisplay();
}


void calculateCurrentSpectrumAndRMS()
{  
  // Spectrum
  fft.window(FFT.HANN);
  fft.forward(audioInput.mix);
  
  for (int ii = 0; ii < numberOfFreqBands; ii++)
  {
    float centerFreq = freqBands[ii];
    float lowerFreq = centerFreq * (1-normalizedBandwidth);
    float upperFreq = centerFreq * (1+normalizedBandwidth);
    
    int lowerFreqIndex = fft.freqToIndex(lowerFreq);
    int upperFreqIndex = fft.freqToIndex(upperFreq);

    float tmp = 0;
    for (int jj = lowerFreqIndex; jj <= upperFreqIndex; jj++)
    {
      tmp = tmp + fft.getBand(jj);
    }
    tmp = tmp / (upperFreqIndex-lowerFreqIndex+1);
    
    averageData[averageCounter][ii] = tmp;
    averageCounter++;
    if (averageCounter == averages)
      averageCounter = 0;
  }
  
  // RMS
  float rms = 0;
  for (int ii = 0; ii < bufferSize; ii++)
  {
    rms = rms + sq(audioInput.mix.get(ii));
  }
  rms = rms / bufferSize;
  rms = sqrt(rms);
  
  averageDataRMS[averageCounterRMS] = rms;
  averageCounterRMS++;
  if (averageCounterRMS == averagesRMS)
    averageCounterRMS = 0;
}


void calculateAverages()
{  
  // Spectrum
  for (int ii = 0; ii < numberOfFreqBands; ii++)
  {  
    averagedData[ii] = 0;
    for (int jj = 0; jj < averages; jj++)
    {
      averagedData[ii] = averagedData[ii] + averageData[jj][ii];
    }
    averagedData[ii] = averagedData[ii]/averages;
    averagedData_dB[ii] = 20*log(averagedData[ii]) + 60;
  }
  
  // RMS
  averagedDataRMS = 0;
  for (int jj = 0; jj < averagesRMS; jj++)
  {
    averagedDataRMS = averagedDataRMS + averageDataRMS[jj];
  }
  averagedDataRMS = averagedDataRMS / averagesRMS;
  averagedDataRMS_dB = 20*log(averagedDataRMS) + 200;
}


void updateDisplay()
{  
  int offsetx = 29;
  int offsety = 190;
  
  background(200);
  stroke(0, 150, 0);
  
  for (int ii = 0; ii < numberOfFreqBands; ii++)
  { 
    if (averagedData_dB[ii] > 0)
    {      
      line(((ii+1)*offsetx)-3, offsety, ((ii+1)*offsetx)-3, offsety - averagedData_dB[ii]);
      line(((ii+1)*offsetx)-2, offsety, ((ii+1)*offsetx)-2, offsety - averagedData_dB[ii]);
      line(((ii+1)*offsetx)-1, offsety, ((ii+1)*offsetx)-1, offsety - averagedData_dB[ii]);
      line(((ii+1)*offsetx)+0, offsety, ((ii+1)*offsetx)+0, offsety - averagedData_dB[ii]);
      line(((ii+1)*offsetx)+1, offsety, ((ii+1)*offsetx)+1, offsety - averagedData_dB[ii]);
      line(((ii+1)*offsetx)+2, offsety, ((ii+1)*offsetx)+2, offsety - averagedData_dB[ii]);
      line(((ii+1)*offsetx)+3, offsety, ((ii+1)*offsetx)+3, offsety - averagedData_dB[ii]);
    }    
  }
  
  stroke(0, 0, 255);
  
  if (averagedDataRMS_dB > 0)
  {
    line(950, offsety, 950, offsety - averagedDataRMS_dB);
  }
}
