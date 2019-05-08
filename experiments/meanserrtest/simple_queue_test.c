
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>


#define NP_ADC_N_SAMPLES 256

uint16_t get_random_data()
{
	uint16_t r = rand();
	return r % 4095;
}

int main()
{
	time_t t;

	srand((unsigned) time(&t));


	uint16_t alldata[NP_ADC_N_SAMPLES];
	float d = 0; 		// Store individual data values
	float mean = 0;
	float serr = 0;
	uint16_t data[2];
	data[0] = data[1] = 0;

	for(uint16_t i=0; i<NP_ADC_N_SAMPLES; i++)
	{
	//	alldata[i] = get_random_data();
		d = (float) get_random_data();
		mean += d;
		serr += d*d;
	}


	mean = mean / (float)NP_ADC_N_SAMPLES;	// Calculate mean
	serr = sqrt( (serr - NP_ADC_N_SAMPLES*mean*mean)/(NP_ADC_N_SAMPLES*(NP_ADC_N_SAMPLES-1.0)) );  // Calculate standard error

	mean = fmin(mean,4095);
	serr = fmax(serr,1);			// Minimum 1 bit error

	data[0] = mean;
	data[1] = serr;

	printf("\nContinuous calculation: mean = %d  serr = %d", data[0], data[1]);

/*	float sd1 = 0;

	for(uint16_t j; j<NP_ADC_N_SAMPLES; j++)
	{
		sd1 += (mean - (float)alldata[j]) * (mean - (float)alldata[j]);

	}

	float sd2 = sd1 / ((float)NP_ADC_N_SAMPLES - 1.0);

	float sd3 = sqrt(sd2);

	float sd4 = sd3 / sqrt((float)NP_ADC_N_SAMPLES);

	uint16_t sd5 = sd4;

	printf("\nStandard method = %f, %f, %f, %f, %d", sd1, sd2, sd3, sd4, sd5);*/




	return 0;



}
