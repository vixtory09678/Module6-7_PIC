/*
    robotics-za.blogspot.com
    https://github.com/vixtory09678/Common_PIC
*/
// local path
#include "lib/24FJ48GA002.h"
#include "lib/BL_Support.h"
#include "const/const_var.h"
#include "const/pin_var.h"
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "math/math_calculate.h"
#include "hw_io/interrupt.h"

#use delay(internal = 8 MHz, clock = 32MHz)
#PIN_SELECT U1RX = PIN_B12 // pin_b14
#PIN_SELECT U1TX = PIN_B13 // pin_b15

#use rs232(UART1, BAUD = 9600, XMIT = PIN_B13, RCV = PIN_B12, timeout = 500)

#include "communication/print_form.h"
#include "communication/protocol.h"
#include "control/motor.h"
#include "control/position_control.h"

void setupPwm();
void setupHardware();
inline void verifyLimitSwitch();
inline void onDataReceive();
inline void onDetectLine();

volatile long pulseX = 0;
volatile long pulseY = 0;
int directZ = GO_FORWARD;

#INT_TIMER1
void TIMER1_isr(void){
    curPos.x = distance(pulseX);
    curPos.y = distance(pulseY);

    gotoXY(setPos.x, setPos.y);
    // onDetectLine();
}

#INT_TIMER3
void TIMER3_isr(void){
    onDetectLine();
}

void main(void){
    setupHardware();
    delay_ms(50);

    while (TRUE){

        long now4 = get_timer4();
        if (now4 > 625){
            servoControl();
            set_timer4(0);
        }

        if (dataCom.cmd == GO_HOME){
            gotoHome();
        }else{
            if (curPos.z == setPos.z){
                driveMotor(M_Z, GO_BREAK, 0);
                if (feedBack.zPosition && !firstFeedBack.zPosition){
                    feedBack.zPosition = FALSE;
                    sentData(TASK_COMPLETE, Z_POSITION);
                }
            }else{
                driveMotor(M_Z, directZ, 0);
                firstFeedBack.zPosition = FALSE;
            }
            motorControl(); // control motor via PID Feedback control
        }
        
        verifyLimitSwitch(); // verify limit switch for set home
        onDataReceive(); // handle when receive data from serial
    }
}

void setupHardware(){
    // setup pin
    set_tris_a(0b10100);
    set_tris_b(0b0011001011110000);

    // setup interrupt & timer
    disable_interrupts(GLOBAL);
    initTimer();
    setupPwm();
    setSerialInterrupt();
    setupEncoderInterrupt();
    enable_interrupts(global);

    // set go home default
    dataCom.cmd = GO_HOME;
    X_HOME = FALSE;
    Y_HOME = FALSE;
    Z_HOME = FALSE;
    servoRotate(DEFAULT_ROTATE);
    servoKeep(GRIPPER_OPEN);
    delay_ms(2000);
}

void setupPwm(){
    // setup pwm drive dc motor
    setup_timer2(TMR_INTERNAL | TMR_DIV_BY_8, TIME_PERIOD_PWM);
    setup_compare(1, COMPARE_PWM | COMPARE_TIMER2);
    setup_compare(2, COMPARE_PWM | COMPARE_TIMER2);

    // setup pwm servo
    setup_compare(3, COMPARE_PWM | COMPARE_TIMER3);
    setup_compare(4, COMPARE_PWM | COMPARE_TIMER3);
}

inline void verifyLimitSwitch(){
    if (input(SWITCH_X) == 0)
        X_HOME = TRUE;

    if (input(SWITCH_Y) == 0)
        Y_HOME = TRUE;

    if (input(SWITCH_Z) == 0)
        Z_HOME = TRUE;
}

BOOLEAN checkTime = FALSE;
inline void onDetectLine(){
    if (input(PIN_B9) == 0){
        if (checkTime == FALSE){
            checkTime = TRUE;
            if (directZ == GO_FORWARD){
                if (curPos.z >= setPos.z)
                    curPos.z = setPos.z;
                else
                    curPos.z = (curPos.z > 5) ? 5 : curPos.z + 1;
            }else{
                if (curPos.z <= setPos.z)
                    curPos.z = setPos.z;
                else
                    curPos.z = (curPos.z < 1) ? 1 : curPos.z - 1;
            }
        }
    }
    else
        checkTime = FALSE;
    
}

inline void onDataReceive() {
    if (isReady) {
        isReady = false;
        dataCom = decodePackage(data);
        float buff = dataCom.data1 / 100.0f;

        switch (dataCom.cmd) {
        case GO_HOME:
            feedBack.goHome = TRUE;
            firstFeedBack.goHome = TRUE;
            X_HOME = FALSE;
            Y_HOME = FALSE;
            Z_HOME = FALSE;
            setPos.x = 0.0;
            setPos.y = 0.0;
            setPos.z = 0;
            directZ = GO_FORWARD;
            gripperRotate.setAngle = DEFAULT_ROTATE;
            gripperKeep.setAngle = GRIPPER_OPEN;
            break;

        case X_POSITION:
            // do something when get SET_POSITION command
            feedBack.xPosition = TRUE;
            firstFeedBack.xPosition = TRUE;

            if (setPos.x == buff)
                sentData(TASK_COMPLETE, X_POSITION);

            setPos.x = buff;
            setPos.x = (setPos.x > MAX_WORK_SPACE_X) ? MAX_WORK_SPACE_X : (setPos.y < MIN_WORK_SPACE_X) ? MIN_WORK_SPACE_X : setPos.x;

            // reset PID
            errMotorX.err = 0f;
            errMotorX.outPID = 0f;
            errMotorX.sumErr = 0f;
            errMotorX.pError = setPos.x - curPos.x;

            break;

        case Y_POSITION:
            // do something when get ROTATE command
            feedBack.yPosition = TRUE;
            firstFeedBack.yPosition = TRUE;

            if (setPos.y == buff)
                sentData(TASK_COMPLETE, Y_POSITION);

            setPos.y = buff;
            setPos.y = (setPos.y > MAX_WORK_SPACE_Y) ? MAX_WORK_SPACE_Y : (setPos.y < MIN_WORK_SPACE_Y) ? MIN_WORK_SPACE_Y : setPos.y;

            
            // reset PID
            errMotorY.err = 0f;
            errMotorY.outPID = 0f;
            errMotorY.sumErr = 0;
            errMotorY.pError = setPos.y - curPos.y;

            break;

        case Z_POSITION:
            // do something when get KEEP command
            feedBack.zPosition = TRUE;
            firstFeedBack.zPosition = TRUE;
            setPos.z = dataCom.data1;

            if (setPos.z > 5)
                setPos.z = 5;
            else if (setPos.z < 1)
                setPos.z = 1;

            if (setPos.z > curPos.z)
                directZ = GO_FORWARD;
            else
                directZ = GO_BACKWARD;

            break;

        case ROTATE:
            // do something when get Z_POSITION command
            feedBack.rotation = TRUE;
            firstFeedBack.rotation = TRUE;

            if (gripperRotate.setAngle == angleToPulse(dataCom.data1))
                sentData(TASK_COMPLETE, ROTATE);
            gripperRotate.setAngle = angleToPulse(dataCom.data1);

            break;

        case KEEP:
            feedBack.keeping = TRUE;
            firstFeedBack.keeping = TRUE;
            if (dataCom.data1 == 1){
                if (gripperKeep.setAngle == DEFAULT_KEEP)){
                    sentData(TASK_COMPLETE, KEEP);
                }
                gripperKeep.setAngle = DEFAULT_KEEP;
            }else{
                if (gripperKeep.setAngle == GRIPPER_OPEN)){
                    sentData(TASK_COMPLETE, KEEP);
                }
                gripperKeep.setAngle = GRIPPER_OPEN;
            }
            break;

        default:
            // do something when get ERROR_DECODE command
            sentData(ERROR_DECODE, 0);
            break;
        }
    }
}