//-*-c++-*-
#ifndef INCLUDED_CreateCommands_h_
#define INCLUDED_CreateCommands_h_

// Create Commands
// Commands unique to Create 2
#ifdef TGT_IS_CREATE2
const unsigned char CREATE_RESET               = 7;
const unsigned char CREATE_POWER               = 133;
const unsigned char CREATE_CLEAN               = 135;
const unsigned char CREATE_MAX                 = 136;
const unsigned char CREATE_MOTOR               = 138;
const unsigned char CREATE_SEEK_DOCK           = 143;
const unsigned char CREATE_PWM_LSD             = 144;
const unsigned char CREATE_PWM_MOTOR           = 146;
const unsigned char CREATE_SCHEDULE_LED        = 162;
const unsigned char CREATE_RAW_LED             = 163;
const unsigned char CREATE_ASCII_LED           = 164;
const unsigned char CREATE_BUTTONS             = 165;
const unsigned char CREATE_SCHEDULE            = 167;
const unsigned char CREATE_DAY_TIME            = 168;
const unsigned char CREATE_STOP	               = 173;
#else
// Commands unique to Create
const unsigned char CREATE_COVER               = 135;
const unsigned char CREATE_DEMO                = 136;
const unsigned char CREATE_LSD                 = 138;
const unsigned char CREATE_COVER_AND_DOCK      = 143;
const unsigned char CREATE_PWM_LSD             = 144;
const unsigned char CREATE_DIGITAL_OUT         = 147;
const unsigned char CREATE_SCRIPT              = 152;
const unsigned char CREATE_SEND_IR             = 151;
const unsigned char CREATE_PLAY_SCRIPT         = 153;
const unsigned char CREATE_SHOW_SCRIPT         = 154;
const unsigned char CREATE_WAIT_TIME           = 155;
const unsigned char CREATE_WAIT_DISTANCE       = 156;
const unsigned char CREATE_WAIT_ANGLE          = 157;
const unsigned char CREATE_WAIT_EVENT          = 158;
#endif
// Common Commands
const unsigned char CREATE_START               = 128;
const unsigned char CREATE_BAUD                = 129;
const unsigned char CREATE_SAFE_OLD            = 130;
const unsigned char CREATE_SAFE                = 131;
const unsigned char CREATE_FULL                = 132;
const unsigned char CREATE_DRIVE               = 137;
const unsigned char CREATE_LEDS                = 139;
const unsigned char CREATE_SONG                = 140;
const unsigned char CREATE_PLAY_SONG           = 141;
const unsigned char CREATE_SENSORS             = 142;
const unsigned char CREATE_DRIVE_DIRECT        = 145;
const unsigned char CREATE_STREAM              = 148;
const unsigned char CREATE_QUERY_LIST          = 149;
const unsigned char CREATE_PAUSE_RESUME_STREAM = 150;

// Create sensor packet code
// Sensor packets unique to create 2
const unsigned char CREATE_SENSOR_GROUP_100          = 100;
const unsigned char CREATE_SENSOR_GROUP_101          = 101;
const unsigned char CREATE_SENSOR_GROUP_106          = 106;
const unsigned char CREATE_SENSOR_GROUP_107          = 107;

const unsigned char CREATE_SENSOR_DIRT               = 15;
const unsigned char CREATE_SENSOR_IR_LEFT            = 52;
const unsigned char CREATE_SENSOR_IR_RIGHT           = 53;
const unsigned char CREATE_SENSOR_ENCODER_LEFT       = 43;
const unsigned char CREATE_SENSOR_ENCODER_RIGHT      = 44;
const unsigned char CREATE_SENSOR_LIGHT_BUMPER       = 45;
const unsigned char CREATE_SENSOR_LIGHT_L_BUMP       = 46;
const unsigned char CREATE_SENSOR_LIGHT_FL_BUMP      = 47;
const unsigned char CREATE_SENSOR_LIGHT_CL_BUMP      = 48;
const unsigned char CREATE_SENSOR_LIGHT_CR_BUMP      = 49;
const unsigned char CREATE_SENSOR_LIGHT_FR_BUMP      = 50;
const unsigned char CREATE_SENSOR_LIGHT_R_BUMP       = 51;
const unsigned char CREATE_SENSOR_CURRENT_L_MOTOR    = 54;
const unsigned char CREATE_SENSOR_CURRENT_R_MOTOR    = 55;
const unsigned char CREATE_SENSOR_CURRENT_MAIN_BRUSH = 56;
const unsigned char CREATE_SENSOR_CURRENT_SIDE_BRUSH = 57;
const unsigned char CREATE_SENSOR_STASIS             = 58;
const unsigned char CREATE_SENSOR_UNUSED_3           = 32;
const unsigned char CREATE_SENSOR_UNUSED_4           = 33;
// Sensor packets unique to create 1
const unsigned char CREATE_SENSOR_UNUSED_1           = 15;
const unsigned char CREATE_SENSOR_DIGITAL_IN         = 32;
const unsigned char CREATE_SENSOR_ANALOG             = 33;

// Common sensor packets
const unsigned char CREATE_SENSOR_GROUP_0            = 0;
const unsigned char CREATE_SENSOR_GROUP_1            = 1;
const unsigned char CREATE_SENSOR_GROUP_2            = 2;
const unsigned char CREATE_SENSOR_GROUP_3            = 3;
const unsigned char CREATE_SENSOR_GROUP_4            = 4;
const unsigned char CREATE_SENSOR_GROUP_5            = 5;
const unsigned char CREATE_SENSOR_GROUP_6            = 6;

const unsigned char CREATE_SENSOR_DROP               = 7;
const unsigned char CREATE_SENSOR_WALL               = 8;
const unsigned char CREATE_SENSOR_CLIFF_LEFT         = 9;
const unsigned char CREATE_SENSOR_CLIFF_FRONT_LEFT   = 10;
const unsigned char CREATE_SENSOR_CLIFF_FRONT_RIGHT  = 11;
const unsigned char CREATE_SENSOR_CLIFF_RIGHT        = 12;
const unsigned char CREATE_SENSOR_VIRTUAL_WALL       = 13;
const unsigned char CREATE_SENSOR_OVERCURRENT        = 14;
const unsigned char CREATE_SENSOR_UNUSED_2           = 16;
const unsigned char CREATE_SENSOR_IR                 = 17;
const unsigned char CREATE_SENSOR_BUTTONS            = 18;
const unsigned char CREATE_SENSOR_DISTANCE           = 19;
const unsigned char CREATE_SENSOR_ANGLE              = 20;
const unsigned char CREATE_SENSOR_CHANGING_STATE     = 21;
const unsigned char CREATE_SENSOR_VOLTAGE            = 22;
const unsigned char CREATE_SENSOR_CURRENT            = 23;
const unsigned char CREATE_SENSOR_BATTERY_TEMP       = 24;
const unsigned char CREATE_SENSOR_BATTERY_CHARGE     = 25;
const unsigned char CREATE_SENSOR_BATTERY_CAPACITY   = 26;
const unsigned char CREATE_SENSOR_WALL_SIGNAL        = 27;
const unsigned char CREATE_SENSOR_CLIFF_L_SIGNAL     = 28;
const unsigned char CREATE_SENSOR_CLIFF_FL_SIGNAL    = 29;
const unsigned char CREATE_SENSOR_CLIFF_FR_SIGNAL    = 30;
const unsigned char CREATE_SENSOR_CLIFF_R_SIGNAL     = 31;
const unsigned char CREATE_SENSOR_CAN_CHARGE         = 34;
const unsigned char CREATE_SENSOR_OI_MODE            = 35;
const unsigned char CREATE_SENSOR_SONG_NUMBER        = 36;
const unsigned char CREATE_SENSOR_SONG_PLAYING       = 37;
const unsigned char CREATE_SENSOR_STREAM_SIZE        = 38;
const unsigned char CREATE_SENSOR_REQ_VELOCITY       = 39;
const unsigned char CREATE_SENSOR_REQ_RADIUS         = 40;
const unsigned char CREATE_SENSOR_REQ_RIGHT_VELOCITY = 41;
const unsigned char CREATE_SENSOR_REQ_LEFT_VELOCITY  = 42;
#endif
