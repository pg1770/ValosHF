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
#define LAP_TIME_MIN 3000 //ms
#define LAP_TIME_MAX 4000

#define START_DELAY = 200 //ms

#define BUFFER_LENGTH 128

#define CORNER_UP 0
#define CORNER_DOWN 1
#define STRAIGHT_LINE 2
#define CORNER_PLANE 3

#define START 0
#define LEARN 1
#define RUN 2

int track_buffer[BUFFER_LENGTH];
unsigned long time_buffer[BUFFER_LENGTH];
int buffer_pos;
int state_per_round;

int car_state;
int track_state;

unsigned long RunTime;


#endif /* TIBI_GERI_H_ */
