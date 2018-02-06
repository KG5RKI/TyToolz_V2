/*! \file ambe.c
  \brief AMBE2+ Hook Functions.
  
  This module contains hooks and replacement functions for AMBE2+.

*/


#include <string.h>

#include "md380.h"
#include "printf.h"
#include "dmesg.h"
#include "version.h"
#include "tooldfu.h"
#include "config.h"
#include "gfx.h"
#include "addl_config.h"


float pow(float base, float ex) {
	// power of 0
	if (ex == 0) {
		return 1;
		// negative exponenet
	}
	else if (ex < 0) {
		return 1 / pow(base, -ex);
		// even exponenet
	}
	else if ((int)ex % 2 == 0) {
		float half_pow = pow(base, ex / 2);
		return half_pow * half_pow;
		//integer exponenet
	}
	else {
		return base * pow(base, ex - 1);
	}
}

float sqrt(float A) {
	float root = 2;
	const float e = 2.71828182846;
	return pow(e, (pow(10.0, 9.0) / root)*(1.0 - (pow(A, -pow(10.0, -9.0)))));
}

#ifndef Max
#define Max(a,b)   (((a) > (b)) ? (a) : (b))
#endif

#ifndef Min
#define Min(a,b)   (((a) < (b)) ? (a) : (b))
#endif

#ifndef sqr
#define sqr(a)   ((a) * (a))
#endif

int max_level;
uint32_t ambe_encode_frame_cnt;

int ambe_encode_thing_hook(char *a1, int a2, int *a3, int a4,
		      short a5, short a6, short a7, int a8){
#ifdef CONFIG_AMBE
  short *s8;
  int i=0;
  int max=0;
  
  s8=(short *)a3; 
  
  for (i=0; i<80; i++) {
    if ( s8[i] > max ) {
      max=s8[i];
    }  
  }
     
  max_level=max;
  ambe_encode_frame_cnt++;
  return ambe_encode_thing(a1,a2,a3,a4,
			   a5,a6,a7,a8);
#else
  return 0xdeadbeef;
#endif
}


/* This hook intercepts calls to ambe_unpack(), which extracts bits
   from the incoming AMBE frame.  The bit buffer is later re-used to
   error-correct the frame, so if we peek at the buffer before
   ambe_unpack() is run we will get the error-corrected bits of the
   preceding packet, but if we peek after ambe_unpack() is called,
   we'll get the bits of the new frame without error correction.
 */
int ambe_unpack_hook(int a1, int a2, char length, int a4){
  /* Dump the previous, error-corrected AMBE frame.  For some reason,
     these aren't decoding properly in DSD. */

#ifdef AMBECORRECTEDPRINT
  short *bits=(short*) a1;
  static int i;

  printf("AMBE2+ Corr: ");
  for(i=0;i<49;i++){
    md380_putc(NULL,bits[i]?'1':'0');
  }
  md380_putc(NULL,'\n');
#endif //AMBECORRECTEDPRINT

#ifdef CONFIG_AMBE
  ambe_unpack(a1,a2,length,a4);
#endif //CONFIG_AMBE

  /* Dump the new, uncorrected AMBE frame.  Bits won't make sense
     until after decoding. */
#ifdef AMBEUNCORRECTEDPRINT
  printf("AMBE2+:  ");
  for(i=0;i<length;i++){
    md380_putc(NULL,bits[i]?'1':'0');
  }
  md380_putc(NULL,'\n');
#endif //AMBEUNCORRECTEDPRINT
  

  return 0;
}


// Big thanks to (Benjamin 'BeRo' Rosseaux) for help with this :)
// also (K8JAS) for help with refining constants for best audio quality

#define SampleRate 8000.0f
float KillDenormal = 0.000000000001f;
#define dBLogFactor 8.68588963807f
#define InputGain  2.0f
#define OutputGain 3.0f
float MinGain = 0.015625;
float MaxGain = 55.0;
#define ClipPoint (0.5507f/OutputGain)
#define TargetPoint (0.45f/OutputGain)
#define NoiseGain  (1.0 / (InputGain*OutputGain))
#define NoiseFloorThresholddB -90.0f
#define AttackTime 0.002f
#define ReleaseTime  0.060f
#define NoiseFloorReleaseTime 1.0
#define LowPassFilterFrequency 3000.0 // Hz
#define HighPassFilterFrequency 100.0 // Hz
int RMSHistorySize = 0x1FF;
int RMSHistoryMask = 0x1FF - 1;


float fState = 1.0f;
float fRMSState = 0.0f;
int fRMSHistoryIndex = 0;
float fRMSHistory[0x1FF] = { 0.0f };
double fAttackCoef = (1.0f - exp(-1.0f / (AttackTime * SampleRate)));
double fReleaseCoef = (1.0f - exp(-1.0f / (ReleaseTime * SampleRate)));
double fNoiseFloorReleaseCoef = 1.0f - exp(-1.0f / (NoiseFloorReleaseTime * SampleRate));
double fNoiseFloorThreshold = exp(NoiseFloorThresholddB / dBLogFactor)*InputGain;
double fLowPassCoef = 2.0f*(sin(3.14159*(LowPassFilterFrequency / SampleRate)));
double fHighPassCoef = 2.0f*(sin(3.14159f*(HighPassFilterFrequency / SampleRate)));

float fLowPassState = 0.0f;
float fHighPassState = 0.0f;

/* The ambe_decode_wav() function decodes AMBE2+ bits to 80 samples
   (160 bytes) of 8kHz audio as signed shorts.  This hook will
   optionally print that data to the dmesg buffer, where it can be
   extracted and recorded on the host.
*/
int ambe_decode_wav_hook(int *a1, signed int eighty, char *bitbuffer,
			 int a4, short a5, short a6, int a7){

  /* This prints the AMBE2+ structure that is to be decoded.  The
     output is decodable with DSD, but it sounds horrible.
   */
#ifdef AMBEPRINT
  int ambestate=OS_ENTER_CRITICAL();
  
  short *bits=(short*) bitbuffer;
  static int i;

  /* I don't know why, but this output can only be decoded by DSD if
     half the frames are dropped.  The trick is to only decode those
     when the sixth paramter is zero, ignoring all the ones where that
     parameter is a one.
     
     Strangely enough, we do not skip half the frames of the WAV
     ouput below.
  */
  if(!a6){
    printf("AMBE2+ Corr: ");
    for(i=0;i<49;i++){
      md380_putc(NULL,bits[i]?'1':'0');
    }
    md380_putc(NULL,'\n');
  }
  OS_EXIT_CRITICAL(ambestate);
#endif //AMBEPRINT

  int toret=0xdeadbeef;
#ifdef CONFIG_AMBE
  //First we call the original function.
  toret=ambe_decode_wav(a1, eighty, bitbuffer,
			a4, a5, a6, a7);
#endif
  float RMS = 0.0f;
  float GainedPeak = 0.0f;
  float Delta = 0.0f;
  float Coef = 0.0f;

  if (global_addl_config.audio_leveling) {
	  short* samples = (short*)a1;
	  for (int i = 0; i < 80; i++)
	  {
		  float samp = samples[i] * 2 * (1.0f / 32768.0f);
		  float inpsamp = samp;

		  fLowPassState = (fLowPassState*(1.0f - fLowPassCoef)) + (inpsamp * fLowPassCoef);
		  fHighPassState = (fHighPassState * (1.0 - fHighPassState)) + (fLowPassState*fHighPassCoef);
		  inpsamp = (fLowPassState - fHighPassState)*InputGain;
		  RMS = sqr(inpsamp);
		  fRMSState = (fRMSState - fRMSHistory[fRMSHistoryIndex & RMSHistoryMask]) + RMS;
		  fRMSHistory[fRMSHistoryIndex & RMSHistoryMask] = RMS;
		  fRMSHistoryIndex = (fRMSHistoryIndex + 1) & RMSHistoryMask;
		  RMS = fRMSState / RMSHistorySize;
		  if (abs(RMS) > 1e-12) {
			  RMS = sqrt(RMS);
		  }

		  float Peak = abs(inpsamp);
		  if (Peak < (fNoiseFloorThreshold*2.0)) {
			  if (RMS < fNoiseFloorThreshold) {
				  fState = (((fState*(1.0 - fNoiseFloorReleaseCoef)) + (NoiseGain*fNoiseFloorReleaseCoef)) + KillDenormal) - KillDenormal;
			  }
		  }
		  else {
			  GainedPeak = Peak*fState;
			  if (GainedPeak > ClipPoint) {
				  fState = TargetPoint / Peak;
			  }
			  else {
				  Delta = ((TargetPoint - GainedPeak) + KillDenormal) - KillDenormal;
				  if (Delta < 0.0) {
					  Coef = fAttackCoef;
				  }
				  else {
					  Coef = fReleaseCoef;
				  }
				  fState = ((fState + (Delta*Coef)) + KillDenormal) - KillDenormal;
			  }
		  }
		  if (fState < MinGain) {
			  fState = MinGain;
		  }
		  else if (fState > MaxGain) {
			  fState = MaxGain;
		  }
		  samples[i] = (Min(Max(round(samp*fState*32768.0f), -32767), 32767)) * 17;
	  }
  }
  

  /* Print the parameters
  printf("ambe_decode_wav(0x%08x, %d, 0x%08x,\n"
    "%d, %d, %d, 0x%08x);\n",
    a1, eighty, bitbuffer,
    a4, a5, a6, a7);
  */

  /* This is very noisy, so we don't enable it by default.  It prints
     the WAV as hex pairs, which will quickly flood the buffer if it
     isn't cleared in time.
   */



 

#ifdef AMBEWAVPRINT
  //Does this really need to be in a critical section?
  int wavstate=OS_ENTER_CRITICAL();
  
  //A1 holds audio as signed LE shorts.
  printf("WAV: ");
  printhex(a1,160);
  printf("\n");
  
  OS_EXIT_CRITICAL(wavstate);
#endif //AMBEWAVPRINT

  
  return toret;
}
