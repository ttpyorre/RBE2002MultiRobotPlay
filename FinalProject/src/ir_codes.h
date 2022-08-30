#ifndef __IR_CODES_H
#define __IR_CODES_H

/**
 * THESE CODES WORK FOR THE REMOTE THAT COMES WITH THE ROMI.
 * 
 * They are NOT for an old TV remote that Lewin uses to test code.
 * :)
 */

#define CHplus      0x0A
#define CHminus     0x08

#define VOLplus     0x02
#define VOLminus    0x00

//Numbers on controller have been tested, these are the results:
#define NUM_0       0x0C
#define NUM_1       0x11
#define NUM_2       0x12
#define NUM_3       0x13
#define NUM_4       0x14
#define NUM_5       0x15
#define NUM_6       0x16
#define NUM_7       0x17
#define NUM_8       0x18
#define NUM_9       0x19

#define STOP        0x06 // STOP/MODE
#define ENTER       0X09 // MIDDLE

#define PREV        0x05 // UP
#define NEXT        0x0D // DOWN

#define CHplus      0x0A // LEFT
#define CHminus     0x08 // RIGHT

#define BACK        0x0E // U-TURN ARROW

#define MUTE        0x01 // RED MIDDLE || BETWEEN VOLMINUS AND VOLPLUS
#define SETUP       0x04

#endif