// Endstops disabled (set to NC in pins.h)

// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
// ported to mbed by R. Bohne (rene.bohne@gmail.com)

#include "mbed.h"
#include "pins.h"
#include "configuration.h"
#include "ThermistorTable.h"


#define DEBUGGING false

#define X_TIME_FOR_MOVE ((float)x_steps_to_take / (x_steps_per_unit*feedrate/60000000))
#define Y_TIME_FOR_MOVE ((float)y_steps_to_take / (y_steps_per_unit*feedrate/60000000))
#define Z_TIME_FOR_MOVE ((float)z_steps_to_take / (z_steps_per_unit*feedrate/60000000))
#define E_TIME_FOR_MOVE ((float)e_steps_to_take / (e_steps_per_unit*feedrate/60000000))


DigitalOut led1(LED1);//x
DigitalOut led2(LED2);//y
DigitalOut led3(LED3);//z
DigitalOut led4(LED4);//e

DigitalOut p_fan(FAN_PIN);

DigitalOut p_X_enable(X_ENABLE_PIN);
DigitalOut p_X_dir(X_DIR_PIN);
DigitalOut p_X_step(X_STEP_PIN);
DigitalIn p_X_min(X_MIN_PIN);
DigitalIn p_X_max(X_MAX_PIN);

DigitalOut p_Y_enable(Y_ENABLE_PIN);
DigitalOut p_Y_dir(Y_DIR_PIN);
DigitalOut p_Y_step(Y_STEP_PIN);
DigitalIn p_Y_min(Y_MIN_PIN);
DigitalIn p_Y_max(Y_MAX_PIN);

DigitalOut p_Z_enable(Z_ENABLE_PIN);
DigitalOut p_Z_dir(Z_DIR_PIN);
DigitalOut p_Z_step(Z_STEP_PIN);
DigitalIn p_Z_min(Z_MIN_PIN);
DigitalIn p_Z_max(Z_MAX_PIN);

DigitalOut p_E_enable(E_ENABLE_PIN);
DigitalOut p_E_dir(E_DIR_PIN);
DigitalOut p_E_step(E_STEP_PIN);

DigitalOut p_heater0(HEATER_0_PIN);

AnalogIn p_temp0(TEMP_0_PIN);


Serial pc(USBTX, USBRX);

Timer timer;

int millis() {
    return timer.read_ms();
}

int micros() {
    return timer.read_us();
}

int max(int a, int b) {
    if (a>b) {
        return a;
    }
    return b;
}

// Takes temperature value as input and returns corresponding analog value from RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inversing the logic from a portion of getTemperature() in FiveD RepRap firmware.
float temp2analog(int celsius) {
    if (USE_THERMISTOR){
        int raw = 0;
        int i;

        for (i=1; i<NUMTEMPS; i++) {
            if (temptable[i][1] < celsius) {
                raw = temptable[i-1][0];
                break;
            }
        }

        // Overflow: Set to last value in the table (25 deg. Celsius)
        if (i == NUMTEMPS) raw = temptable[i-1][0];

        return raw;
    } 
}

// calculated by hand
float analog2temp(int raw) {
    if (USE_THERMISTOR) {
        int celsius = 0;
        int i;

        for (i=1; i<NUMTEMPS; i++) {
            if (temptable[i][0]  > raw) {
                celsius  = temptable[i-1][1];
                break;
            }
        }

        // Overflow: Set to last value in the table (25 deg. Celsius)
        if (i == NUMTEMPS) celsius = temptable[i-1][1];

        return celsius;
    } 
}

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0 -> G1
// G1  - Coordinated Movement X Y Z E
// G4  - Dwell S<seconds> or P<milliseconds>
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for current temp to reach target temp.

//Custom M Codes
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M86  - If Endstop is Not Activated then Abort Print. Specify X and/or Y
// M92  - Set axis_steps_per_unit - same syntax as G92
// M93  - Read previous_micros

//Stepper Movement Variables
bool direction_x, direction_y, direction_z, direction_e;
int previous_micros=0, previous_micros_x=0, previous_micros_y=0, previous_micros_z=0, previous_micros_e=0, previous_millis_heater;
int x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take;
float destination_x =0.0, destination_y = 0.0, destination_z = 0.0, destination_e = 0.0;
float current_x = 0.0, current_y = 0.0, current_z = 0.0, current_e = 0.0;
float x_interval, y_interval, z_interval, e_interval; // for speed delay
float feedrate = 1500, next_feedrate;
float time_for_move;
int gcode_N, gcode_LastN;
bool relative_mode = false;  //Determines Absolute or Relative Coordinates
bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

int x_steps_remaining;
int y_steps_remaining;
int z_steps_remaining;
int e_steps_remaining;

// comm variables
#define MAX_CMD_SIZE 256
char cmdbuffer[MAX_CMD_SIZE];
char serial_char;
int serial_count = 0;
bool comment_mode = false;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//manage heater variables
int target_raw = 0;
int current_raw;

//Inactivity shutdown variables
int previous_millis_cmd=0;
int max_inactive_time = 0;

//timer.read_us overflows every 30 seconds, so we want to reset everything...
void reset_timers() {
    previous_micros = 0;
    previous_micros_x = 0;
    previous_micros_y = 0;
    previous_micros_z = 0;
    previous_micros_e = 0;

    timer.stop();
    timer.reset();
    timer.start();
}


void check_x_min_endstop() {
    if (X_MIN_PIN != NC) {
        if (!direction_x) {
            if (p_X_min.read() != ENDSTOPS_INVERTING) {
                x_steps_remaining=0;
            }
        }
    }
}

void check_y_min_endstop() {
    if (Y_MIN_PIN != NC) {
        if (!direction_y) {
            if (p_Y_min.read() != ENDSTOPS_INVERTING) {
                y_steps_remaining=0;
            }
        }
    }
}

void check_z_min_endstop() {
    if (Z_MIN_PIN != NC) {
        if (!direction_z) {
            if (p_Z_min.read() != ENDSTOPS_INVERTING) {
                z_steps_remaining=0;
            }
        }
    }
}




void manage_heater() {

    if (TEMP_0_PIN != NC) {
        current_raw = 0;
        for(int i=0;i<3;i++)
        {
            int _raw = p_temp0.read_u16();
            if((current_raw == 65535) && (_raw==65535))
            {
               //do nothing
            }
            else if((current_raw == 65535) && (_raw<65535))
            {
                current_raw = _raw;
            }
            else
            {
                long l = current_raw + _raw;
                l = l/2;
                current_raw = (int) l;
            }
        }
        //pc.printf("currentRaw: %d \t targetRaw: %d\n", current_raw, target_raw);
        
    
        if(current_raw == 65535)
        {
           pc.printf("thermistor disconnected!!!\n");
           p_heater0 = 0;
        }
        else
        {
            
        if((target_raw >0) && (current_raw > target_raw))
        {
            p_heater0 = 1;
            //pc.printf("currentRaw: %d \t targetRaw: %d\n", current_raw, target_raw);
        }
        else
        {
          p_heater0 = 0;
        }
        }
        
    }

/*
    if (TEMP_0_PIN != NC) {
        current_raw = (p_temp0.read_u16() >> 6) ;

        if (USE_THERMISTOR) {// If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target,
            current_raw = 0xFFFF - current_raw; // this switches it up so that the reading appears lower than target for the control logic.
        }

        if (current_raw >= target_raw) {
            p_heater0 = 0;
        } else {
            p_heater0 = 1;
        }
    }
*/

}


void do_x_step() {
    if (X_STEP_PIN != NC) {
        p_X_step = 1;
        wait_us(2);
        p_X_step = 0;
        //wait_us(2);
        previous_micros_x = micros();
    }
}

void do_y_step() {
    if (Y_STEP_PIN != NC) {
        p_Y_step = 1;
        wait_us(2);
        p_Y_step = 0;
        //wait_us(2);
        previous_micros_y = micros();
    }
}

void do_z_step() {
    if (Z_STEP_PIN != NC) {
        p_Z_step = 1;
        wait_us(2);
        p_Z_step = 0;
        //wait_us(2);
        previous_micros_z = micros();
    }
}

void do_e_step() {
    if (E_STEP_PIN != NC) {
        p_E_step = 1;
        wait_us(2);
        p_E_step = 0;
        //wait_us(2);
        previous_micros_e = micros();
    }
}


void disable_x() {
    if (X_ENABLE_PIN != NC) {
        p_X_enable = !X_ENABLE_ON;
    }
    led1=0;
}

void disable_y() {
    if (Y_ENABLE_PIN != NC) {
        p_Y_enable = !Y_ENABLE_ON;
    }
    led2=0;
}

void disable_z() {
    if (Z_ENABLE_PIN != NC) {
        p_Z_enable = !Z_ENABLE_ON;
    }
    led3=0;
}

void disable_e() {
    if (E_ENABLE_PIN != NC) {
        p_E_enable = !E_ENABLE_ON;
    }
    led4=0;
}

void  enable_x() {
    if (X_ENABLE_PIN != NC) {
        p_X_enable = X_ENABLE_ON;
    }
}

void  enable_y() {
    if (Y_ENABLE_PIN != NC) {
        p_Y_enable = Y_ENABLE_ON;
    }
}

void  enable_z() {
    if (Z_ENABLE_PIN != NC) {
        p_Z_enable = Z_ENABLE_ON;
    }
}

void  enable_e() {
    if (E_ENABLE_PIN != NC) {
        p_E_enable = E_ENABLE_ON;
    }
}

void kill(int debug) {

/*
    if (HEATER_0_PIN != NC) {
        p_heater0 = 0;
    }
*/

    disable_x();
    disable_y();
    disable_z();
    disable_e();

    if (PS_ON_PIN != NC) {
        //pinMode(PS_ON_PIN,INPUT);
    }

    while (1) {
        switch (debug) {
            case 1:
                pc.printf("Inactivity Shutdown, Last Line: ");
                break;
            case 2:
                pc.printf("Linear Move Abort, Last Line: ");
                break;
            case 3:
                pc.printf("Homing X Min Stop Fail, Last Line: ");
                break;
            case 4:
                pc.printf("Homing Y Min Stop Fail, Last Line: ");
                break;
        }
        pc.printf("%s \n",gcode_LastN);
        wait(5); // 5 Second delay
    }
}

void manage_inactivity(int debug) {
    if ( (millis()-previous_millis_cmd) >  max_inactive_time ) {
        if (max_inactive_time) {
            kill(debug);
        }
    }
}


void linear_move() { // make linear move with preset speeds and destinations, see G0 and G1
    //Determine direction of movement
    if (destination_x > current_x) {
        p_X_dir = !INVERT_X_DIR;
    } else {
        p_X_dir = INVERT_X_DIR;
    }

    if (destination_y > current_y) {
        p_Y_dir = !INVERT_Y_DIR;
    } else {
        p_Y_dir = INVERT_Y_DIR;
    }

    if (destination_z > current_z) {
        p_Z_dir = !INVERT_Z_DIR;
    } else {
        p_Z_dir = INVERT_Z_DIR;
    }

    if (destination_e > current_e) {
        p_E_dir = !INVERT_E_DIR;
    } else {
        p_E_dir = INVERT_E_DIR;
    }

    //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
    if (x_steps_remaining) enable_x();
    if (y_steps_remaining) enable_y();
    if (z_steps_remaining) enable_z();
    if (e_steps_remaining) enable_e();

    check_x_min_endstop();
    check_y_min_endstop();
    check_z_min_endstop();

    previous_millis_heater = millis();

    while (x_steps_remaining + y_steps_remaining + z_steps_remaining + e_steps_remaining > 0) { // move until no more steps remain
        if (x_steps_remaining>0) {
            if ((micros()-previous_micros_x) >= x_interval) {
                do_x_step();
                x_steps_remaining--;
            }
            check_x_min_endstop();
            led1 = 1;
        } else {
            led1 = 0;
            wait_us(2);
        }

        if (y_steps_remaining>0) {
            if ((micros()-previous_micros_y) >= y_interval) {
                do_y_step();
                y_steps_remaining--;
            }
            check_y_min_endstop();
            led2=1;
        } else {
            led2=0;
            wait_us(2);
        }

        if (z_steps_remaining>0) {
            if ((micros()-previous_micros_z) >= z_interval) {
                do_z_step();
                z_steps_remaining--;
            }
            check_z_min_endstop();
            led3=1;
        } else {
            led3=0;
            wait_us(2);
        }

        if (e_steps_remaining>0) {
            if ((micros()-previous_micros_e) >= e_interval) {
                do_e_step();
                e_steps_remaining--;
                led4=1;
            }
        } else {
            led4=0;
            wait_us(2);
        }

        if ( (millis() - previous_millis_heater) >= 500 ) {
            manage_heater();
            previous_millis_heater = millis();

            manage_inactivity(2);
        }

        wait_us(2);
    }

    led1=0;
    led2=0;
    led3=0;
    led4=0;

    if (DISABLE_X) disable_x();
    if (DISABLE_Y) disable_y();
    if (DISABLE_Z) disable_z();
    if (DISABLE_E) disable_e();

    // Update current position partly based on direction, we probably can combine this with the direction code above...
    if (destination_x > current_x) current_x = current_x + x_steps_to_take/x_steps_per_unit;
    else current_x = current_x - x_steps_to_take/x_steps_per_unit;
    if (destination_y > current_y) current_y = current_y + y_steps_to_take/y_steps_per_unit;
    else current_y = current_y - y_steps_to_take/y_steps_per_unit;
    if (destination_z > current_z) current_z = current_z + z_steps_to_take/z_steps_per_unit;
    else current_z = current_z - z_steps_to_take/z_steps_per_unit;
    if (destination_e > current_e) current_e = current_e + e_steps_to_take/e_steps_per_unit;
    else current_e = current_e - e_steps_to_take/e_steps_per_unit;
}




void ClearToSend() {
    previous_millis_cmd = millis();
    pc.printf("ok\n");
}


void FlushSerialRequestResend() {
    pc.printf("Resend: %d\n",(gcode_LastN+1));
    //char cmdbuffer[100]="Resend:";
    //ltoa(gcode_LastN+1, cmdbuffer+7, 10);
    //pc.flush();
    //pc.printf(cmdbuffer);
    ClearToSend();
}


//#define code_num (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL))
//inline void code_search(char code) { strchr_pointer = strchr(cmdbuffer, code); }
float code_value() {
    return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL));
}

long code_value_long() {
    return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10));
}

bool code_seen(char code_string[]) {
    return (strstr(cmdbuffer, code_string) != NULL);    //Return True if the string was found
}

bool code_seen(char code) {
    strchr_pointer = strchr(cmdbuffer, code);
    return (strchr_pointer != NULL);  //Return True if a character was found
}

void get_coordinates() {
    if (code_seen('X')) destination_x = (float)code_value() + relative_mode*current_x;
    else destination_x = current_x;                                                       //Are these else lines really needed?
    if (code_seen('Y')) destination_y = (float)code_value() + relative_mode*current_y;
    else destination_y = current_y;
    if (code_seen('Z')) destination_z = (float)code_value() + relative_mode*current_z;
    else destination_z = current_z;
    if (code_seen('E')) destination_e = (float)code_value() + (relative_mode_e || relative_mode)*current_e;
    else destination_e = current_e;
    if (code_seen('F')) {
        next_feedrate = code_value();
        if (next_feedrate > 0.0) feedrate = next_feedrate;
    }

    //Find direction
    if (destination_x >= current_x) direction_x=1;
    else direction_x=0;
    if (destination_y >= current_y) direction_y=1;
    else direction_y=0;
    if (destination_z >= current_z) direction_z=1;
    else direction_z=0;
    if (destination_e >= current_e) direction_e=1;
    else direction_e=0;


    if (min_software_endstops) {
        if (destination_x < 0) destination_x = 0.0;
        if (destination_y < 0) destination_y = 0.0;
        if (destination_z < 0) destination_z = 0.0;
    }

    if (max_software_endstops) {
        if (destination_x > X_MAX_LENGTH) destination_x = X_MAX_LENGTH;
        if (destination_y > Y_MAX_LENGTH) destination_y = Y_MAX_LENGTH;
        if (destination_z > Z_MAX_LENGTH) destination_z = Z_MAX_LENGTH;
    }

    if (feedrate > max_feedrate) feedrate = max_feedrate;
}

void process_commands() {
    unsigned long codenum; //throw away variable

    if (code_seen('N')) {
        gcode_N = code_value_long();
        if (gcode_N != gcode_LastN+1 && (strstr(cmdbuffer, "M110") == NULL) ) {
            gcode_LastN=0;
            pc.printf("ok");
            //if(gcode_N != gcode_LastN+1 && !code_seen("M110") ) {   //Hmm, compile size is different between using this vs the line above even though it should be the same thing. Keeping old method.
            //pc.printf("Serial Error: Line Number is not Last Line Number+1, Last Line:");
            //pc.printf("%d\n",gcode_LastN);
            //FlushSerialRequestResend();
            return;
        }

        if (code_seen('*')) {
            int checksum = 0;
            int count=0;
            while (cmdbuffer[count] != '*') checksum = checksum^cmdbuffer[count++];

            if ( (int)code_value() != checksum) {
                //pc.printf("Error: checksum mismatch, Last Line:");
                //pc.printf("%d\n",gcode_LastN);
                //FlushSerialRequestResend();
                return;
            }
            //if no errors, continue parsing
        } else {
            //pc.printf("Error: No Checksum with line number, Last Line:");
            //pc.printf("%d\n",gcode_LastN);
            //FlushSerialRequestResend();
            return;
        }

        gcode_LastN = gcode_N;
        //if no errors, continue parsing
    } else { // if we don't receive 'N' but still see '*'
        if (code_seen('*')) {
            //pc.printf("Error: No Line Number with checksum, Last Line:");
            //pc.printf("%d\n",gcode_LastN);
            return;
        }
    }

    //continues parsing only if we don't receive any 'N' or '*' or no errors if we do. :)

    if (code_seen('G')) {
        switch ((int)code_value()) {
            case 0: // G0 -> G1
            case 1: // G1
                reset_timers();//avoid timer overflow after 30 seconds
                get_coordinates(); // For X Y Z E F
                x_steps_to_take = abs(destination_x - current_x)*x_steps_per_unit;
                y_steps_to_take = abs(destination_y - current_y)*y_steps_per_unit;
                z_steps_to_take = abs(destination_z - current_z)*z_steps_per_unit;
                e_steps_to_take = abs(destination_e - current_e)*e_steps_per_unit;
                //printf(" x_steps_to_take:%d\n", x_steps_to_take);


                time_for_move = max(X_TIME_FOR_MOVE,Y_TIME_FOR_MOVE);
                time_for_move = max(time_for_move,Z_TIME_FOR_MOVE);
                time_for_move = max(time_for_move,E_TIME_FOR_MOVE);

                if (x_steps_to_take) x_interval = time_for_move/x_steps_to_take;
                if (y_steps_to_take) y_interval = time_for_move/y_steps_to_take;
                if (z_steps_to_take) z_interval = time_for_move/z_steps_to_take;
                if (e_steps_to_take) e_interval = time_for_move/e_steps_to_take;


                x_steps_remaining = x_steps_to_take;
                y_steps_remaining = y_steps_to_take;
                z_steps_remaining = z_steps_to_take;
                e_steps_remaining = e_steps_to_take;


                if (DEBUGGING) {
                    pc.printf("destination_x: %f\n",destination_x);
                    pc.printf("current_x: %f\n",current_x);
                    pc.printf("x_steps_to_take: %d\n",x_steps_to_take);
                    pc.printf("X_TIME_FOR_MOVE: %f\n",X_TIME_FOR_MOVE);
                    pc.printf("x_interval: %f\n\n",x_interval);

                    pc.printf("destination_y: %f\n",destination_y);
                    pc.printf("current_y: %f\n",current_y);
                    pc.printf("y_steps_to_take: %d\n",y_steps_to_take);
                    pc.printf("Y_TIME_FOR_MOVE: %f\n",Y_TIME_FOR_MOVE);
                    pc.printf("y_interval: %f\n\n",y_interval);

                    pc.printf("destination_z: %f\n",destination_z);
                    pc.printf("current_z: %f\n",current_z);
                    pc.printf("z_steps_to_take: %d\n",z_steps_to_take);
                    pc.printf("Z_TIME_FOR_MOVE: %f\n",Z_TIME_FOR_MOVE);
                    pc.printf("z_interval: %f\n\n",z_interval);

                    pc.printf("destination_e: %f\n",destination_e);
                    pc.printf("current_e: %f\n",current_e);
                    pc.printf("e_steps_to_take: %d\n",e_steps_to_take);
                    pc.printf("E_TIME_FOR_MOVE: %f\n",E_TIME_FOR_MOVE);
                    pc.printf("e_interval: %f\n\n",e_interval);
                }

                linear_move(); // make the move
                ClearToSend();
                return;
            case 4: // G4 dwell
                codenum = 0;
                if (code_seen('P')) codenum = code_value(); // milliseconds to wait
                if (code_seen('S')) codenum = code_value()*1000; // seconds to wait
                previous_millis_heater = millis();  // keep track of when we started waiting
                while ((millis() - previous_millis_heater) < codenum ) manage_heater(); //manage heater until time is up
                break;
            case 90: // G90
                relative_mode = false;
                break;
            case 91: // G91
                relative_mode = true;
                break;
            case 92: // G92
                if (code_seen('X')) current_x = code_value();
                if (code_seen('Y')) current_y = code_value();
                if (code_seen('Z')) current_z = code_value();
                if (code_seen('E')) current_e = code_value();
                break;
           case 93: // G93
                pc.printf("previous_micros:%d\n", previous_micros);
                pc.printf("previous_micros_x:%d\n", previous_micros_x);
                pc.printf("previous_micros_y:%d\n", previous_micros_y);
                pc.printf("previous_micros_z:%d\n", previous_micros_z);
                break;

        }
    }

    if (code_seen('M')) {

        switch ( (int)code_value() ) {
            case 104: // M104
                
                if (code_seen('S'))
                {
                     
                    target_raw = temp2analog(code_value());
                    //pc.printf("target_raw: %d\n ", target_raw);
                }
                break;
            case 105: // M105
                pc.printf("ok T:");
                if (TEMP_0_PIN != NC) {
                    pc.printf("%f\n", analog2temp( (p_temp0.read_u16())  ));
                } else {
                    pc.printf("0.0\n");
                }
                if (!code_seen('N')) return; // If M105 is sent from generated gcode, then it needs a response.
                break;
            case 109: // M109 - Wait for heater to reach target.
                if (code_seen('S')) target_raw = temp2analog(code_value());
                previous_millis_heater = millis();
                while (current_raw < target_raw) {
                    if ( (millis()-previous_millis_heater) > 1000 ) { //Print Temp Reading every 1 second while heating up.
                        pc.printf("ok T:");
                        if (TEMP_0_PIN != NC) {
                            pc.printf("%f\n", analog2temp(p_temp0.read_u16()));
                        } else {
                            pc.printf("0.0\n");
                        }
                        previous_millis_heater = millis();
                    }
                    manage_heater();
                }
                break;
            case 106: //M106 Fan On
                p_fan = 1;
                break;
            case 107: //M107 Fan Off
                p_fan = 0;
                break;
            case 80: // M81 - ATX Power On
                //if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,OUTPUT); //GND
                break;
            case 81: // M81 - ATX Power Off
                //if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT); //Floating
                break;
            case 82:
                relative_mode_e = false;
                break;
            case 83:
                relative_mode_e = true;
                break;
            case 84:
                disable_x();
                disable_y();
                disable_z();
                disable_e();
                break;
            case 85: // M85
                code_seen('S');
                max_inactive_time = code_value()*1000;
                break;
            case 86: // M86 If Endstop is Not Activated then Abort Print
                if (code_seen('X')) {
                    if (X_MIN_PIN != NC) {
                        if ( p_X_min == ENDSTOPS_INVERTING ) {
                            kill(3);
                        }
                    }
                }
                if (code_seen('Y')) {
                    if (Y_MIN_PIN != NC) {
                        if ( p_Y_min == ENDSTOPS_INVERTING ) {
                            kill(4);
                        }
                    }
                }
                break;
            case 92: // M92
                if (code_seen('X')) x_steps_per_unit = code_value();
                if (code_seen('Y')) y_steps_per_unit = code_value();
                if (code_seen('Z')) z_steps_per_unit = code_value();
                if (code_seen('E')) e_steps_per_unit = code_value();
                break;
        }

    }

    ClearToSend();
}


void get_command() {
    if ( pc.readable() ) {
        serial_char = pc.getc();

        if (serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) ) {
            if (!serial_count) {
                return; //empty line
            }
            cmdbuffer[serial_count] = 0; //terminate string

            process_commands();

            comment_mode = false; //for new command
            serial_count = 0; //clear buffer
            //Serial.println("ok");
        } else {
            if (serial_char == ';') {
                comment_mode = true;
            }
            if (!comment_mode) {
                cmdbuffer[serial_count++] = serial_char;
            }
        }
    }


}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    pc.baud(BAUDRATE);
    pc.printf("start\n");//RepRap
    //pc.printf("A:\n");//HYDRA
}

void loop() {
    get_command();
    
    manage_heater();
    
    manage_inactivity(1); //shutdown if not receiving any new commands
}

int main() {
    timer.start();
    setup();

    while (1) {
        loop();
    }
}
