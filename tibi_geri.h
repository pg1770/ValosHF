/*
 * tibi_geri.h
 *
 *  Created on: 2014.04.18.
 *      Author: Tibor
 */

#ifndef TIBI_GERI_H_
#define TIBI_GERI_H_

#define DEBUG_MOD

#define DELTA_T_BEFORE_CORNER_BREAK 1400
#define DELTA_T_BEFORE_CORNER_ACC 700
#define DELTA_T_BEFORE_STRAIGHT 1000

//1/2ms os a timerunk
#define LAP_TIME_MIN 0 //4500*2 //ms per kör 2500 as motor fesz mellett
#define LAP_TIME_MAX 7000*2 // kb a min 1,5 szerese

#define BUFFER_LENGTH 128

#define NARROW_CORNER 4
#define WIDE_CORNER 5

#define CORNER_LEFT 0
#define CORNER_RIGHT 1
#define STRAIGHT_LINE 2
#define CORNER_PLANE 3
#define CORNER

#define START 0
#define LEARN 1
#define RUN 2

#define WAIT_BEFORE_LEARN 1000 //ms
//#define WAIT_BEFORE_NEW_STATE 100

#define CONST_VEL 1800
#define CORNER_MAX_VOL 1800
#define CORNER_BREAK 1400
#define WIDE_MAX_VOL 1800
#define STRAIGHT_MAX_VOL 3000


#define THRESHOLD_CENTER 32000

#define THRESHOLD_STRAIGHT 1000
#define THRESHOLD_CORNER 2000

//#define THRESHOLD_NARROW 1500
//#define THRESHOLD_WIDE 2000

int actual_narrow_vol;
int actual_wide_vol;
int actual_straight_vol;

unsigned long prev_time;

int track_buffer[BUFFER_LENGTH];
unsigned long time_buffer[BUFFER_LENGTH];
int buffer_pos;
int state_per_round;

int car_state;
int track_state;

int round;

int corner_break_time;
int corner_acc_time;


int period_length;
int min_period_index;
int max_period_index;

int next_state;

//egy periodusra a state-ek
int period_buffer[20];
//a period_buffer[x-dik state je utan mennyi idovel kovetkezik a kovetkezo
unsigned long period_times[20];
unsigned long prev_period_times[20];

int period_index;



#endif /* TIBI_GERI_H_ */
