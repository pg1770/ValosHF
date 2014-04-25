/*
 * tibi_geri.h
 *
 *  Created on: 2014.04.18.
 *      Author: Tibor
 */

#ifndef TIBI_GERI_H_
#define TIBI_GERI_H_

#define DEBUG_MOD


#define THRESHOLD_UP 33000 //lefele meno kanyar
#define THRESHOLD_DOWN 31000 //felele meno kanyar

#define DELAY 150

#define CONST_VEL 2500
//1/2ms os a timerunk
#define LAP_TIME_MIN 3500*2 //ms per kör 2500 as motor fesz mellett
#define LAP_TIME_MAX 5200*2 // kb a min 1,5 szerese

#define START_DELAY = 200 //ms

#define BUFFER_LENGTH 128

#define CORNER_UP 0
#define CORNER_DOWN 1
#define STRAIGHT_LINE 2
#define CORNER_PLANE 3

#define START 0
#define LEARN 1
#define RUN 2

#define WAIT_BEFORE_LEARN 200 //ms

int track_buffer[BUFFER_LENGTH];
unsigned long time_buffer[BUFFER_LENGTH];
int buffer_pos;
int state_per_round;

int car_state;
int track_state;

int round;

int period_length;
int min_period_index;


#endif /* TIBI_GERI_H_ */
