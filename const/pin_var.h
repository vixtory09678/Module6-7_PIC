#ifndef PIN_VAR_H
#define PIN_VAR_H

// -------------- PIN --------------------
// servo rotate
#PIN_SELECT oc3 = PIN_B14
// servo keep
#PIN_SELECT oc4 = PIN_B15

// PIN_A3 is DTR

// limit switch
#define SWITCH_X PIN_A0
#define SWITCH_Y PIN_A1
#define SWITCH_Z PIN_B11

//motor X
#define DIR_M1_IN1 PIN_A2
#define DIR_M1_IN2 PIN_A4
#PIN_SELECT oc1 = PIN_B0

// motor y
#define DIR_M2_IN2 PIN_B2
#define DIR_M2_IN1 PIN_B1
#PIN_SELECT oc2 = PIN_B3

// motor z
#define DIR_M3_IN1 PIN_B8
#define DIR_M3_IN2 PIN_B10

// encoder X axis
#define INT_MOTORX_A PIN_B4
#define INT_MOTORX_B PIN_B5
#PIN_SELECT INT1 = INT_MOTORX_A

// encoder Y axis
#define INT_EXT0_PIN PIN_B7
#define INT_MOTORY_B PIN_B6

//========================================

#endif