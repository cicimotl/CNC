/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)       // add to get a lot more serial output.

#define VERSION              (2)                      // firmware version
#define BAUD                 (115200)                 // How fast is the Arduino talking?(BAUD Rate of Arduino)
#define MAX_BUF              (64)                     // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (200)                    // depends on your stepper motor.  most are 200.
#define STEPS_PER_MM         (STEPS_PER_TURN*16/0.8)  // (400*16)/0.8 with a M5 spindle
#define MAX_FEEDRATE         (100)
#define MIN_FEEDRATE         (1)
#define NUM_AXIES            (4)


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;

