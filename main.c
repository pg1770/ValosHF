/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2009 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 *******************************************************************************
 *
 * File      main.c
 *
 * Author    Milan Brejl
 *
 * Version   1.0
 *
 * Date      26-Nov-2009
 *
 * Brief     Quick start with the Self-Driven Slot Car development
 *
 *******************************************************************************
 * History:
 * 1.0 (26-Nov-2009) Initial version
 *******************************************************************************
 *
 * This application includes initialization code, drivers and macros enabling to
 * quickly start with the development of a Self-Driven Slot Car.
 *
 * This application is configured to run under the SlotCarBootloader framework:
 * - linker file commands the linker to use flash memory up to the bootloader
 * - interrupt vector table is redirected to RAM where a copy of the application
 *   vector table is created.
 *
 * At the current state, the application starts the car running at a constant
 * motor voltage, samples and logs all analog signals (X, Y, and Z accelerations,
 * track voltage and motor current) to a text file on the SD card.
 * X and Y accelerations are filtered by two different filter types. Take it
 * as an axample of filtering and decide on your own if these filteres are
 * suitable for use.
 * Head lights indicate the detection of a right or a left curve. Break lights
 * indicate detection of a curve beginning, based on a simple thresholding of
 * Y-acceleration.
 *
 *
 * Notes:
 * 1) The MCU MCF51JM64 is configured to run at 48MHz CPU clock and 24MHz BUS
 *    clock.
 * 2) The motor is controlled in one direction only, using a motorVoltage
 *    unsigned int variable (range 0 to 6000) and a PWM signal of corresponding
 *    duty-cycle.
 *    In order to control the motor in both directions (active breaking), the
 *    initialization of the TPM2 module needs to be modified for a generation of
 *    2 PWM signals. For more information refer to the MC33931 H-bridge manual.
 * 3) The ran-time data logging to a file on the SD card is handled by ChaN's
 *    FAT System Module (http://elm-chan.org/fsw/ff/00index_e.html)
 *    and there available example low-level MMC/SD/SDHC via SPI routines
 *    ported to ColdFire v1.
 *
 ******************************************************************************/
#include <hidef.h>          /* for EnableInterrupts macro */
#include "derivative.h"     /* include peripheral declarations */
#include "ramvector.h"      /* redirects interrupt vectors into RAM */
#include "slotcar.h"        /* slot car HW related macros and routines */
#include "ff.h"             /* access to FAT file system on SD card */
#include "tibi_geri.h"


/******************************************************************************
 * Constants and macros
 ******************************************************************************/
#define LOG_BUFFER_SIZE               32
#define ACCXFILT_THRESHOLD_LEFT    35000 /* depends on the particular device */
#define ACCXFILT_THRESHOLD_RIGHT   29000 /* depends on the particular device */
#define ACCYFILT_THRESHOLD         37000 /* depends on the particular device */

#define SD_CARD_EXISTS

/******************************************************************************
 * Global variables
 ******************************************************************************/
/* accelerometer */
unsigned short accX, accY, accZ; /* 12-bit unsigned samples */
unsigned short accXFilt, accYFilt; /* 16-bit unsigned filtered values */

/* motor */
unsigned short motorVoltage; /* range 0 to 6000 */
unsigned short motorCurrent; /* 12-bit unsigned sample */

/* track voltage */
unsigned short trackVoltage; /* 12-bit unsigned sample */

/* global time */
volatile unsigned long timeCounter; /* global free-running 1/2ms counter */


/* logBuffer */
struct
{
	unsigned long timeCounter;
	unsigned short accXFilt;
	unsigned short accYFilt;
	unsigned short accZ;
	unsigned short trackVoltage;
	unsigned short motorCurrent;
}logBuffer[LOG_BUFFER_SIZE];
byte idxRead = 0; /* logBuffer read index */
byte idxWrite = 0; /* logBuffer write index */

/* SD card */
FATFS fileSystem; /* FAT driver File System Object */
FIL file; /* log file File Object */
FIL logFile; //külön kellene rá ifdef ha ráérünk
FIL junkLogFile; //külön kellene rá ifdef ha ráérünk

/******************************************************************************
 * Functions
 ******************************************************************************/

/* FilterHalfBand8Lynn function */
/*
 * The FilterHalfBand8Lynn function filters the input 12-bit sample
 * using an 8-level multirate half-band system based on Lynn filters.
 * Every 128th call the function returns a nonzero value which is a
 * sample of the filtered and decimated output signal.
 */
unsigned int FilterHalfBand8Lynn(unsigned int input) {
	static unsigned char step;
	static unsigned int a0, a1, a2;
	static unsigned int b, b0, b1, b2;
	static unsigned int c, c0, c1, c2;
	static unsigned int d, d0, d1, d2;
	static unsigned int e, e0, e1, e2;
	static unsigned int f, f0, f1, f2;
	static unsigned int g, g0, g1, g2;
	static unsigned int h0, h1, h2;
	unsigned char step0;
	unsigned int output = 0;

	step0 = step;
	step++;
	switch (step0 ^ step) {
	case 1:
		a2 = input >> 1;
		b = a0 + a1 + a2; /* 1st level filter output, 13-bit range */
		a0 = a2;
		break;

	case 3:
		a1 = input;

		b2 = b >> 1;
		c = b0 + b1 + b2; /* 2nd level filter output, 14-bit range */
		b0 = b2;
		break;

	case 7:
		a1 = input;
		b1 = b;

		c2 = c >> 1;
		d = c0 + c1 + c2; /* 3rd level filter output, 15-bit range */
		c0 = c2;
		break;

	case 15:
		a1 = input;
		b1 = b;
		c1 = c;

		d2 = d >> 1;
		e = d0 + d1 + d2; /* 4th level filter output, 16-bit range */
		d0 = d2;
		break;

	case 31:
		a1 = input;
		b1 = b;
		c1 = c;
		d1 = d;

		e2 = e >> 2;
		f = e0 + e1 + e2; /* 5th level filter output, 16-bit range */
		e0 = e2;
		break;

	case 63:
		a1 = input;
		b1 = b;
		c1 = c;
		d1 = d;
		e1 = e >> 1;

		f2 = f >> 2;
		g = f0 + f1 + f2; /* 6th level filter output, 16-bit range */
		f0 = f2;
		break;

	case 127:
		a1 = input;
		b1 = b;
		c1 = c;
		d1 = d;
		e1 = e >> 1;
		f1 = f >> 1;

		g2 = g >> 2;
		h2 = g0 + g1 + g2; /* 7th level filter output, 16-bit range */
		g0 = g2;
		break;

	case 255:
		a1 = input;
		b1 = b;
		c1 = c;
		d1 = d;
		e1 = e >> 1;
		f1 = f >> 1;
		g1 = g >> 1;

		output = (h0 >> 2) + (h1 >> 1) + (h2 >> 2); /* 8th level filter output, 16-bit range */
		h0 = h1;
		h1 = h2;
		break;
	}

	return (output);
}

/******************************************************************************
 * Interrupt Handlers
 ******************************************************************************/

/* Real Time Counter Interrupt (10ms period) */
interrupt VectorNumber_Vrtc void Vrtc_isr(void)
{
#ifdef SD_CARD_EXISTS

	/* Update iodisk_sd internal timers */
	disk_timerproc();
	/* Stare measured values into the logBuffer */
	logBuffer[idxWrite].timeCounter = timeCounter;
	logBuffer[idxWrite].accXFilt = accXFilt;
	logBuffer[idxWrite].accYFilt = accYFilt;
	logBuffer[idxWrite].accZ = accZ;
	logBuffer[idxWrite].trackVoltage = trackVoltage;
	logBuffer[idxWrite].motorCurrent = motorCurrent;

	if(++idxWrite == LOG_BUFFER_SIZE)
	{
		idxWrite = 0;
	}
#endif

	/* Light head lights according to accX */
	if(accXFilt > ACCXFILT_THRESHOLD_LEFT)
	{
		SET_LED_HL_ON;
	}
	else if(accXFilt < ACCXFILT_THRESHOLD_RIGHT)
	{
		SET_LED_HR_ON;
	}
	else
	{
		SET_LED_HL_OFF;
		SET_LED_HR_OFF;
	}

	/* Light break lights according to accY */
	if(accYFilt > ACCYFILT_THRESHOLD)
	{
		SET_LED_BL_ON;
		SET_LED_BR_ON;
	}
	else
	{
		SET_LED_BL_OFF;
		SET_LED_BR_OFF;
	}

	/* Clear the interrupt flag */
	RTCSC_RTIF = 1;
}

/* Timer/PWM Interrupt (1/2ms interrupt period) */
interrupt VectorNumber_Vtpm2ovf void Vtpm2ovf_isr(void)
{
	/* clear interrupt flag */
	TPM2SC_TOF = 0;

	/* update PWM duty-cycle according to the global variable motorVoltage */
	SET_MOTOR_VOLTAGE(motorVoltage);

	/* Start sequence of analog inputs sampling */
	START_CONV(MOTOR_CURRENT);

	/* Update the global 1/2ms timeCounter */
	timeCounter++;
}

/* ADC Conversion Complete Interrupt (sequence of 5 interrupts every 1/2ms)*/
interrupt VectorNumber_Vadc void Vadc_isr(void)
{
	unsigned int output;

	/*
	 * Read the new sample using READ_ADC_SAMPLE macro.
	 * The reading also clears the interrupt flag.
	 */
	switch(ADCSC1_ADCH)
	{
		case MOTOR_CURRENT:
		motorCurrent = READ_ADC_SAMPLE;
		START_CONV(ACC_X);
		break;
		case ACC_X:
		accX = READ_ADC_SAMPLE;
		START_CONV(ACC_Y);
		/* 8-Level Half Band filter */
		if ((output = FilterHalfBand8Lynn(accX)) != 0)
		{
			accXFilt = output;
		}
		break;
		case ACC_Y:
		accY = READ_ADC_SAMPLE;
		START_CONV(ACC_Z);
		/* EWMA filter */
		accYFilt = accYFilt - (accYFilt>>4) + accY;
		break;
		case ACC_Z:
		accZ = READ_ADC_SAMPLE;
		if(GET_INT1)
		{
			INT1_TO_ADC_CONNECT;
			START_CONV(TRACK_VOLTAGE_1);
		}
		else if(GET_INT2)
		{
			INT2_TO_ADC_CONNECT;
			START_CONV(TRACK_VOLTAGE_2);
		}
		break;
		case TRACK_VOLTAGE_1:
		case TRACK_VOLTAGE_2:
		INTx_TO_ADC_DISCONNECT;
		trackVoltage = READ_ADC_SAMPLE;
		break;
	}
}

/* Keyboard Interrupt */
interrupt VectorNumber_Vkeyboard void Vkeyboard_isr(void)
{
	KBI1SC_KBIE = 0; /* while not used, disable KBI interrupts */

	/* clear interrupt flag */
	KBI1SC_KBACK=1;
}


/*****************************************************************************/

void learn()
{
	int k;
	// beallitjuk a max period indexet
	if( (timeCounter >= LAP_TIME_MAX) && (max_period_index == -1))
	{
		max_period_index = buffer_pos;
		f_printf(&junkLogFile, "timecount >= LAP_TIME_MAX ...\n max_period_index %d \n", max_period_index);
	}
	
	// ha mar biztosan mentunk 2 kort, akkor megprobaljuk megkeresni
	// a periodust
	if (timeCounter >= LAP_TIME_MAX*2)
	{
		//f_printf(&junkLogFile, "timecount >= LAP_TIME_MIN*2 \n");
		period_length = find_period_length(min_period_index, max_period_index);
		
		for( k = 0; k < period_length; k++)
		{
			period_buffer[k] = track_buffer[k+1];
			period_times[k] = time_buffer[k+2] - time_buffer[k+1];
		}
		
		
		f_printf(&junkLogFile, "timecount >= LAP_TIME_MIN*2 \n period_length %d period_index %d state %d\n", 
			period_length, period_index,period_buffer[buffer_pos]);   
	}

	// ha mar megallapitottuk a palyaperiodus hosszat
	// akkor kepesek vagyunk kiszamolni, hany egesy kort tettunk meg
	if( period_length != -1)
	{
		round = buffer_pos/period_length;
		f_printf(&junkLogFile, "period_length != -1 \n round %d \n", round);
	}

	// folyamatosan logoljuk, melyik palyallapotban tartunk
	if(feel_track_and_time_buffers(idxRead))
	{
		prev_time = timeCounter;
	}

	// ha megallapitottuk a palyaperiodust ES mar mentunk 3 kort,
	// akkor nekikezdunk a versenynek
	if(round >= 3 && period_length != -1)
	{
		car_state = RUN;
		period_index = (buffer_pos-1)%period_length;
		f_printf(&junkLogFile, "RUN state set timeCounter %d\n",timeCounter);
	}
}

void run()
{
	// uj allapotba leptunk
	if(feel_track_and_time_buffers(idxRead))
	{
		period_times[period_index] = timeCounter - prev_time;
		prev_time = (timeCounter + prev_time)/2;
		
		period_index++;	
		if(period_index == period_length)
			period_index = 0;
		
		f_printf(&junkLogFile, "period_index %d state %d timeCounter %d period_buffer[period_index] %d\n",
				period_index,
				period_buffer[period_index],
				timeCounter,
				period_buffer[period_index]);
		
		//a kovetkezo allpotban vagyunk
		switch (period_buffer[period_index]){
			case(STRAIGHT_LINE):
				if(actual_straight_vol < STRAIGHT_MAX_VOL)
				{
					actual_straight_vol += 200;	
				}
				motorVoltage = actual_straight_vol;
				f_printf(&junkLogFile, "new state actual_straight_vol %d timeCounter %d\n",
						actual_straight_vol,
						timeCounter);
				break;
			case(CORNER_LEFT):
			case(CORNER_RIGHT):
				motorVoltage = CORNER_MAX_VOL;
				f_printf(&junkLogFile, "new state actual_narrow_vol %d timeCounter %d\n", 
						actual_narrow_vol,
						timeCounter);
				break;
				
		}
		
	} //end of if(feel_track_and_time_buffers(idxRead))
	//epp nem lepunk uj allapotba
	else
	{
	    // a szakaszra megfelelo sebessegel megyunk be
		next_state = period_buffer[(period_index + 1)%period_length];
		
		switch(period_buffer[period_index])
		{
			case (STRAIGHT_LINE):
				switch(next_state)
				{
					case(CORNER_LEFT):
					case(CORNER_RIGHT):

						if( (timeCounter - prev_time) >= (period_times[period_index] - DELTA_T_BEFORE_CORNER_BREAK ))
						{			
							motorVoltage = CORNER_BREAK;
						}
					
						if( (timeCounter - prev_time) >= (period_times[period_index] - DELTA_T_BEFORE_CORNER_ACC ))
						{
						motorVoltage = CORNER_MAX_VOL;
						}
				
						break;	
				}				
				break;
			case(CORNER_LEFT):
			case(CORNER_RIGHT):
			
				switch(next_state)
				{
					case(STRAIGHT_LINE):
						if( (timeCounter - prev_time) >= (period_times[period_index] - DELTA_T_BEFORE_STRAIGHT ))
						{
							motorVoltage = actual_straight_vol;
						}
					break;	
				}
				break;
		} //end of switch(period_buffer[period_index])
		
	} // end of else of if(feel_track_and_time_buffers(idxRead))
}




/*
 * Egy pályaperiódus hosszát adja vissza. Ha még nem tudtuk megállapítani, akkor -1 et.
 * Periódus hossza alatt a egy kör által tartalmazott pályaelemeket értjük (egyenes, emelkedõ kanyar,
 * lejtõ kanyar)
 * Akkor hívjuk meg elõször, ha már a pálya elértük a vélhetõ periodicitás maximuma kétszeresénél
 * fellépõ pozíciót is. (kétszeres azért jó, mert akkor két kört biztosan megtettünk,
 * így könnyebben és biztosabban találhatunk periódust)
 *
 * A track bufferben keres periodicitást.
 * Paramétere az elsõ kör vélhetõ végének a minimuma, mint a pozícióbufferben
 * elfoglalt állapot indexe.
 *
 * pl: 5 db állapotot vettünk fel amíg el nem értük a minimum vélhetõ pályavéget (amit megadtunk
 * define-nal) és 7 db állapotot, amíg el nem értük a maximumot, akkor a paraméterek
 * (mivel zero-based a tömbök indexelése) 4 és 6
 */

int find_period_length(int mini, int maxi)
{
  int match = -1;
  int i, j, n;
  int counter = 0;

  if(buffer_pos < (maxi*2+1))
  {
	return match;
  }

  for (n = maxi; n >= mini; --n)
  {
    for (i = buffer_pos - n; i >= n; --i)
    {
      for (j = 0; j <= n; ++j)
      {
        if (track_buffer[j] == track_buffer[j + i])
        {
          ++counter;
        }
      }

      if ((counter-1) == n && i == n)
        match = n;
      else counter = 0;

    }
  }

  return match;
}

/*
 * Ezt folyamatosan hívnánk ha van új adat, azaz a gyorsulásmérõ írt be új gyorsulási adatokat
 * A track bufferbe írja be az új pozíciót
 * Azaz jegyezzük, hogy eddig milyen pályaelemmel találkoztunk (egyenes, emelkedõ kanyar,
 * lejtõ kanyar)
 * visszateresi ertek, hogy lett e uj allapot
 */

int feel_track_and_time_buffers(int idxRead)
{
	
	//ha van uj allapotunk
	if((logBuffer[idxRead].accXFilt > (THRESHOLD_CENTER + THRESHOLD_CORNER)) &&
			track_buffer[buffer_pos] != CORNER_LEFT)
	{
			buffer_pos++;
			if (buffer_pos >= BUFFER_LENGTH)
			{
				f_printf(&logFile, "Buffer overflow" );
				buffer_pos = 0;
			}
			track_buffer[buffer_pos] = CORNER_LEFT;
			time_buffer[buffer_pos] = timeCounter;
			f_printf(&logFile, "buf_pos %d state %d time %d motorV %d\n", 
					buffer_pos, 
					CORNER_LEFT, 
					timeCounter, 
					motorVoltage);
			return 1;
			
	}

	else if((logBuffer[idxRead].accXFilt < (THRESHOLD_CENTER - THRESHOLD_CORNER)) &&
			track_buffer[buffer_pos] != CORNER_RIGHT)
	{
			buffer_pos++;
			if (buffer_pos >= BUFFER_LENGTH)
			{
				f_printf(&logFile, "Buffer overflow" );
				buffer_pos = 0;
			}
			track_buffer[buffer_pos] = CORNER_RIGHT;
			time_buffer[buffer_pos] = timeCounter;
			f_printf(&logFile, "buf_pos %d state %d time %d motorV %d\n", 
					buffer_pos, 
					CORNER_RIGHT, 
					timeCounter, 
					motorVoltage);
			return 1;

	}
	else if ((track_buffer[buffer_pos] != STRAIGHT_LINE) &&
			(logBuffer[idxRead].accXFilt < (THRESHOLD_CENTER + THRESHOLD_STRAIGHT)) &&
			(logBuffer[idxRead].accXFilt > (THRESHOLD_CENTER - THRESHOLD_STRAIGHT)))
	{
			buffer_pos++;
			if (buffer_pos >= BUFFER_LENGTH)
			{
				f_printf(&logFile, "Buffer overflow" );
				buffer_pos = 0;
			}
			track_buffer[buffer_pos] = STRAIGHT_LINE;
			time_buffer[buffer_pos] = timeCounter;
			f_printf(&logFile, "buf_pos %d state %d time %d motorV %d\n", 
					buffer_pos, 
					STRAIGHT_LINE, 
					timeCounter, 
					motorVoltage);
			return 1;
	}
	return 0;
}

/*
 * accY atlagolasa
 * parameterek: az atlagolando adat indexe, es az atlagolas hossza
 * visszateresi ertek a szamitott atlag
 */
/*
unsigned short average_accY(int index, int num)
{
	int i = 0;
	int sum = 0;

	// ha meg nincs eleg adatunk h atlagoljunk,
	// nehogy seg fault legyen
	if (index < num)
	{
		return track_buffer[index].average_accY;
	}

	for(i = 0, sum = 0; i < num ; i++)
	{
		sum += track_buffer[index - i].average_accY;
	}

	return sum/num;

}
*/

/******************************************************************************
 * Main
 ******************************************************************************/
void main(void) {
	/* This needs to be here when running in bootloader framework */
	RedirectInterruptVectorsToRAM();

	/******************* Device and Board Initialization ***********************/
	SlotCarInit();

	/*-------------------------------------------*/
	/* inits */
	car_state = START;
	min_period_index = -1;
	max_period_index = -1;
	period_length = -1;
	min_period_index = 1;

	actual_straight_vol = CONST_VEL;
	/* enable interrupts */
	EnableInterrupts;

	/* break lights on */
	SET_LED_BL_ON;
	SET_LED_BR_ON;

#ifdef SD_CARD_EXISTS
	/* create new file on SD card for data logging */
	word val, i = 0;
	byte k ;
	byte idx;
	char fileName[] =        "00000000.CSV";
	char logFileName[] =     "slog0.txt";
	char junkLogLifeName[] = "junk0.txt";
	f_mount(0, &fileSystem);
	do
	{
		idx = 8;
		val = i++;
		do
		{
			fileName[--idx] = (char)(val % 10 + '0');
			val /= 10;
		}
		while (idx && val);
	}while(FR_EXIST == f_open(&file, fileName, FA_CREATE_NEW | FA_WRITE));

	k = 0;
	do
	{
		idx = 4;
		logFileName[idx] = (char)(k + '0');
		k++;
	}
	while(FR_EXIST == f_open(&logFile, logFileName, FA_CREATE_NEW | FA_WRITE));

	
	k = 0;
	do
	{
		idx = 4;
		junkLogLifeName[idx] = (char)(k + '0');
		k++;
	}
	while(FR_EXIST == f_open(&junkLogFile, junkLogLifeName, FA_CREATE_NEW | FA_WRITE));
	
	/* write header line */
	f_printf(&file, "%s\n", "timeCounter;accXFilt;accYFilt;accZ;trackVoltage;motorCurrent");


#endif

	/* break lights off */
	SET_LED_BL_OFF;
	SET_LED_BR_OFF;

	/* enable motor */
	motorVoltage = CONST_VEL;

	MOTOR_ENABLE;
	
	f_printf(&junkLogFile, "Start \n");

	/*************************** Background Loop *******************************/
	while (1) {
		/* Check motor fault status */
		if(GET_MOTOR_FAULT_STATUS == MOTOR_STATUS_FAULT)
		{
			motorVoltage = 0;
			MOTOR_DISABLE;
			f_printf(&logFile, "Motor failed\n");
		}


#ifdef SD_CARD_EXISTS
		/* logBuffer not empty? */


		if(idxRead != idxWrite)
		{
			//atlagolas
			//average_accY(idxRead,4);


			// Az auto allapotai
			
			switch(car_state)
			{
			f_printf(&logFile, "car_state %d\n", car_state);

			// START: adott idonyit varunk, mielott elkezenenk logolni/feldolgozni
			// adatainkat
			case START:
				round = 0;
				if(timeCounter >=  WAIT_BEFORE_LEARN)
					car_state = LEARN;
				break;

			// LEARN: tanulas fazis
			case LEARN:
				
				learn();
				break;

			//RUN: azaz mar versenyzunk
			case RUN:
				
				run();
				break; //case RUN breakje
				
			} //end of switch(car_state)
			/*
			 * A mereseket mindig logoljuk, barmitol fuggetlenul
			 */
			
			/* Format log data from buffer to CSV text */
			f_printf(&file,"%d;%d;%d;%d;%d;%d\n", logBuffer[idxRead].timeCounter,
					logBuffer[idxRead].accXFilt,
					logBuffer[idxRead].accYFilt,
					logBuffer[idxRead].accZ,
					logBuffer[idxRead].trackVoltage,
					logBuffer[idxRead].motorCurrent);
			if(++idxRead == LOG_BUFFER_SIZE)
			{
				idxRead = 0;
			}
		}
		
		else
		{
			/* Put data physically to the SD card */
			f_sync(&file);
			f_sync(&logFile);
			f_sync(&junkLogFile);

		}
#else
		volatile unsigned long i;
		for (i = 0; i < 1000; i++)
		;
#endif
	}
#ifdef SD_CARD_EXISTS
	/* never gets here to close the file and dismount SD card - don't care, because: */
	f_close(&file); /* calls f_sync and then clears file object */
	f_close(&logFile);
	f_close(&junkLogFile);
	f_mount(0, NULL); /* clears fat system object, hardware not touched */
#endif
}



/*****************************************************************************/
/* Freescale  is  not  obligated  to  provide  any  support, upgrades or new */
/* releases  of  the Software. Freescale may make changes to the Software at */
/* any time, without any obligation to notify or provide updated versions of */
/* the  Software  to you. Freescale expressly disclaims any warranty for the */
/* Software.  The  Software is provided as is, without warranty of any kind, */
/* either  express  or  implied,  including, without limitation, the implied */
/* warranties  of  merchantability,  fitness  for  a  particular purpose, or */
/* non-infringement.  You  assume  the entire risk arising out of the use or */
/* performance of the Software, or any systems you design using the software */
/* (if  any).  Nothing  may  be construed as a warranty or representation by */
/* Freescale  that  the  Software  or  any derivative work developed with or */
/* incorporating  the  Software  will  be  free  from  infringement  of  the */
/* intellectual property rights of third parties. In no event will Freescale */
/* be  liable,  whether in contract, tort, or otherwise, for any incidental, */
/* special,  indirect, consequential or punitive damages, including, but not */
/* limited  to,  damages  for  any loss of use, loss of time, inconvenience, */
/* commercial loss, or lost profits, savings, or revenues to the full extent */
/* such  may be disclaimed by law. The Software is not fault tolerant and is */
/* not  designed,  manufactured  or  intended by Freescale for incorporation */
/* into  products intended for use or resale in on-line control equipment in */
/* hazardous, dangerous to life or potentially life-threatening environments */
/* requiring  fail-safe  performance,  such  as  in the operation of nuclear */
/* facilities,  aircraft  navigation  or  communication systems, air traffic */
/* control,  direct  life  support machines or weapons systems, in which the */
/* failure  of  products  could  lead  directly to death, personal injury or */
/* severe  physical  or  environmental  damage  (High  Risk Activities). You */
/* specifically  represent and warrant that you will not use the Software or */
/* any  derivative  work of the Software for High Risk Activities.           */
/* Freescale  and the Freescale logos are registered trademarks of Freescale */
/* Semiconductor Inc.                                                        */
/*****************************************************************************/
