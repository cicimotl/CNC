/*
 * Copyright (C) 2017 Ciara Mo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//---------------------------------Constant-------------------------------------
//#define VERBOSE         (1)                                                     //add to get a lot more serial output
#define VERSION         (1.0)
#define BAUD            (115200)
#define MAX_BUF         (64)
#define STEPS_PER_TURN  (100)
#define STEPS_PER_MM    (STEPS_PER_TURN*16/0.8)
#define MAX_FEEDRATE    (1000)
#define MIN_FEEDRATE    (1)
#define NUM_AXIES       (2)
//------------------------------------------------------------------------------



//-----------------------------------STRUCTS------------------------------------
typedef struct {                                                                //for line()
    long delta;                                                                 //# of steps to move
    long absdelta;          
    long over;                                                                  //for dx/dy bresenham calculations
} Axis;

typedef struct {                                                                //for setting up stepper motors i.e. setup()
    int step_pin;
    int dir_pin;
    int enable_pin;
    int limit_switch_pin;
} Motor;
//------------------------------------------------------------------------------



//------------------------------------GLOBALS-----------------------------------
Axis a[NUM_AXIES];                                                              //for line()
Axis atemp;                                                                     //for line()
Motor motors[NUM_AXIES];    

char buffer[MAX_BUF];                                                           //where we store the message until we get a ';'
int buffer_left;                                                                //how much is in the buffer

//speeds
float fr = 0;                                                                   //human version
long step_delay;                                                                //machine version
float px, py, pe;                                                               //position

//settings
char mode_abs = 1;                                                              //absolute mode ON
long line_number = 0;
//------------------------------------------------------------------------------



//----------------------------------METHODS-------------------------------------
/********
    - a thousand (1000) microseconds in a millisecond 
    - a million (1,000,000) microseconds in a second
    - there are 1000 milliseconds in a second.
    - INPUT ms how many milliseconds to wait
********/
void pause(long ms) {                                                           //delay for an appropriate time period
    delay(ms/1000);                                                             //Pauses the program for the amount of time in miliseconds
    delayMicroseconds(ms%1000);                                                 //Pauses the program for the amount of time in microseconds
                                                                                //delayMicroseconds() does not work for values >~16k
}

/********
    - set the FEEDRATE (step motors will move)
    - INPUT nfr, the new speed in steps/second
********/
void feedrate(float nfr) {
    nfr = nfr * STEPS_PER_MM/60;
    if (fr==nfr) {return;}                                                      //if same as last time, break

    if (nfr > MAX_FEEDRATE || nfr < MIN_FEEDRATE) {
        Serial.print(F("New FEEDRATE must be greater than "));
        Serial.print(MIN_FEEDRATE);
        Serial.print(F("Steps/s and less than "));
        Serial.print(MAX_FEEDRATE);
        Serial.println(F("steps/s."));
        return;
    }
    step_delay = MAX_FEEDRATE / nfr;
    fr = nfr;
}

/********
    - INPUT npx for new x position
    - INPUT npy for new y position for motor1
    - INPUT npe for new y position for motor2
 ********/
void position(float npx, float npy) {                                           //Set the logical position
    px = npx;                                                                   //here to test for sanity of positioning
    py = npy;
    pe = npy;
}

/********
    - Supports movement with both styles of Motor Shield
    - INPUT newx, the destination x position
    - INPUT newy, the destination y position for motor1
 ********/
void onestep(int motor) {
#ifdef VERBOSE
    char *letter = "XYZE";
    Serial.print(letter[]);
#endif
    
    digitalWrite(motors[motor].step_pin, HIGH);                                 //digitalWrite() Write a HIGH or a LOW value to a digital pin.
    digitalWrite(motors[motor].step_pin, LOW);                                  //HIGH->ON LOW->OFF
}   

/********
    - Uses Bresenham's line algorithm to move both motors
    - INPUT newx, the new destination x position
    - INPUT newy, the new destination y position for motor1
    - INPUT newpe, the new destination y position for motor2
 ********/
void line(float newx, float newy) {
    a[0].delta = (newx - px) * STEPS_PER_MM;                                    //reminding: delta = #s of steps to move
    a[1].delta = (newy - py) * STEPS_PER_MM;
    
    long i, j, maxsteps = 0;
    
    for (i = 0; i < NUM_AXIES; i++) {
        a[i].absdelta = abs(a[i].delta);
        a[i].over = 0;                                                          //reminding: over = dx/dy bresenham calculations
        
        if (maxsteps < a[i].absdelta) {maxsteps = a[i].absdelta;}
        
        digitalWrite(motors[i].dir_pin, a[i].delta > 0? HIGH:LOW);              //set the direction once per movement
    }
    
    long dt = MAX_FEEDRATE / 5000;
    long accel = 1;
    long steps_to_accel = dt - step_delay;
    if (steps_to_accel > maxsteps / 2) {steps_to_accel = maxsteps / 2;}
    
    long steps_to_decel = maxsteps - steps_to_accel;
    
    Serial.print("START ");
    Serial.print(dt);
    Serial.print("STOP ");
    Serial.println(step_delay);
    
    Serial.print("Accel until ");
    Serial.print(steps_to_accel);
    Serial.print("Decel after ");
    Serial.print(steps_to_decel);
    Serial.print("Total ");
    Serial.println(maxsteps);
    
#ifdef VERBOSE
    Serial.println(F("START >"));
#endif
    
    for (i = 0; i < maxsteps; ++i) {
        for (j = 0; j < NUM_AXIES; ++j) {
            a[j].over += a[j].absdelta;
            if (a[j].over >= maxsteps) {
                a[j].over-= maxsteps;
                
                digitalWrite(motors[j].step_pin, HIGH);
                digitalWrite(motors[j].step_pin, LOW);
            }
        }
        
        if (i < steps_to_accel) {dt -= accel;}
        if (i >= steps_to_decel) {dt+= accel;}
        delayMicroseconds(dt);
    }
    
#ifdef VERBOSE
    Serial.println(F("< DONE!"));
#endif
    
    position(newx, newy);
    where();
}

//returns angle of dy/dx as a value from 0-2Pi
static float atan3(float dy, float dx) {
    float a = atan2(dy, dx);
    if (a < 0) {a = (PI * 2.0) + a;}
    return a;
}

/********
    - Look for character /code/ in the buffer and read the float that immediately follows it
    - RETURN the value found. If not found, /val/ is returned
    - INPUT code, the character to look for
    - INPUT val, the return value if /code/ is not found
 ********/
float parsenumber(char code, float val) {
    char *ptr = buffer;
    while(ptr && *ptr && ptr < buffer + buffer_left) {
        if (*ptr == code) {
            return atof(ptr + 1);
        }
        ptr = strchr(ptr, ' ') + 1;
    }
    return val;
}

/********
    - Write a string followed by a float to the serial line
    - Convenient for debugging
    - INPUT code, the string
    - INPUT val, the float
 ********/
void output(const char *code, float val) {
    Serial.print(code);
    Serial.print(val);
    Serial.print(" ");
}

/********
    - Print the current POSITION, FEEDRATE, and ABSIOLUTE/RELATIVE mode
 ********/
void where() {
    output("X", px);
    output("Y", py);
    output("F", fr / STEPS_PER_MM * 60);
    Serial.println(mode_abs? "ABS":"REL");
}

/********
    - Display helpful information
 ********/
void help() {
    Serial.print(F("CNC MACHINE V "));
    Serial.println(VERSION);
    Serial.println(F("BY MICHAEL BORDEN and CIARA MO"));
    Serial.println(F("Commands: "));
    Serial.println(F("G00/G01 [X/Y(steps)] [F(FEEDRATE]; - Linear move"));
    Serial.println(F("G04 P[Seconds]; - Delay"));
    Serial.println(F("G90; - Absolute mode"));
    Serial.println(F("G91; - Relative mode"));
    Serial.println(F("G92 [X/Y(steps)]; - Change logical POSITION"));
    Serial.println(F("M18; - Disable motors"));
    Serial.println(F("M100; - This help message"));
    Serial.println(F("M114; - Report POSITION and FEEDRATE"));
    Serial.println(F("One G or M command per line"));
    Serial.println(F("All commands must end with a newline."));
}

/********
    - Read the input buffer and find any recognized commands. 
    - 1 G or M command per Line
 ********/
void ProcessCommand() {
    int cmd = parsenumber('G', -1);
    switch(cmd) {
        case 0:
        case 1: {                                                               //line
            feedrate(parsenumber('F', fr));
            line(parsenumber('X', (mode_abs?px:0)) + (mode_abs?0:px),
                 parsenumber('Y', (mode_abs?py:0)) + (mode_abs?0:py));
            break;
        }
        case 2:
        case 4: pause(parsenumber('P', 0)*1000); break;                         //dwell
        case 90: mode_abs = 1; break;                                           //G90 - absolute mode
        case 91: mode_abs = 0; break;                                           //G91 - relative mode
        case 92: position(parsenumber('X', 0),                                  //set logical position
                          parsenumber('Y', 0));
                          break;
        default: break;
    }
    
    cmd = parsenumber('M', -1);
    switch(cmd) {
        case 17: motor_enable(); break;
        case 18: motor_disable(); break;
        case 100: help(); break;
        case 114: where(); break;
        default: break;
    }
}
/********
    - Prepares the input buffer to receive a new message and tells the serial 
      connected device it is ready for more.
 ********/
void ready() {
    buffer_left = 0;                                                            //clear input buffer
    Serial.print(F(">"));                                                       //signal ready to receive input
}

void motor_setup() {
    motors[0].step_pin=2;
    motors[0].dir_pin=5;
    motors[0].enable_pin=8;
    motors[0].limit_switch_pin=9;

    motors[1].step_pin=3;
    motors[1].dir_pin=6;
    motors[1].enable_pin=8;
    motors[1].limit_switch_pin=10;

    motors[2].step_pin=4;
    motors[2].dir_pin=7;
    motors[2].enable_pin=8;
    motors[2].limit_switch_pin=11;

    motors[3].step_pin=12;
    motors[3].dir_pin=13;
    motors[3].enable_pin=8;
    motors[3].limit_switch_pin=11;
    
    int i;
    for (i = 0; i < NUM_AXIES; ++i) {
        pinMode(motors[i].step_pin, OUTPUT);                                    //pinMode() - Configures the specified pin to behave either as an input or an output.
        pinMode(motors[i].dir_pin, OUTPUT);                                     //set the motor pin & scale
        pinMode(motors[i].enable_pin, OUTPUT);
    }
}

void motor_enable() {
    int i;
    for (i = 0; i < NUM_AXIES; ++i) {
        digitalWrite(motors[i].enable_pin, LOW);                                 //enable_pin set as OUTPUT, so LOW -> ON
    }
}

void motor_disable() {
    int i;
    for (i = 0; i < NUM_AXIES; ++i) {
        digitalWrite(motors[i].enable_pin, HIGH);                              //enable_pin set as OUTPUT, so HIGH -> OFF
    }
}

/********
    - Startup code
    - Only run once
 ********/
void setup() {
    Serial.begin(BAUD);                                                         //open coms i.e. the serial monitor in Audrino
    
    motor_setup();
    motor_enable();
    
    where();                                                                    //for debugging purpose
    help();                                                                     //display useful information
    position(0, 0);                                                             //set starting position
    feedrate(1000);                                                             //set default feedrate
    ready();
}

/********
    - After setup() the machine will repeat loop() forever
 ********/
void loop() {                                                                   //available() - Get the number of bytes (characters) available for reading from the serial port
    while(Serial.available() > 0) {                                             //if something is available
        char c = Serial.read();                                                 //get it
        Serial.print(c);                                                        //repeat it back for acknowledgement
        
        if (buffer_left < MAX_BUF - 1) {buffer[buffer_left++] = c;}             //store it
        
        if (c == '\n') {                                                        //entire message received
            buffer[buffer_left] = 0;                                            //end the buffer so string functions work right
            Serial.print(F("\r\n"));                                            //echo a return character for humans
            ProcessCommand();                                                   //process the command
            ready();
        }
    }
}
//------------------------------------------------------------------------------


