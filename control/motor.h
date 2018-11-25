#ifndef MOTOR_H
#define MOTOR_H

#include "var.h"

void driveMotor(uint8_t motor, uint8_t dir, int speed);
void gotoXY(float x, float y);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void pwmWrite(uint8_t motor, float duty);
long angleToPulse(int degree);

typedef struct ErrorControl{
    float err;
    float sumErr;
    float pError;
    float outPID;
} errorControl;

typedef struct Coordinates{
    float x;
    float y;
    int z;
} Position;

typedef struct Gripper{
    int curAngle;
    int error;
    int setAngle;
} gripper;

gripper gripperRotate = {DEFAULT_ROTATE, 0, DEFAULT_ROTATE};
gripper gripperKeep = {GRIPPER_OPEN, 0, GRIPPER_OPEN};

volatile errorControl errMotorX = {0f, 0f, 0f, 0f};
volatile errorControl errMotorY = {0f, 0f, 0f, 0f};

volatile Position curPos = {0.0, 0.0, 0};
volatile Position setPos = {0.0, 0.0, 0};

long angleToPulse(int degree){
    long pulse = degree;
    if (pulse > 180)
        pulse = 180;
    else if (pulse < 0)
        pulse = 0;
    pulse = map(degree, 0, 180, 32, 127);
    return pulse;
}

void servoRotate(int pulse){
    set_pwm_duty(3, pulse);
}

void servoKeep(int pulse){
    set_pwm_duty(4, pulse);
}

void driveMotor(uint8_t motor, uint8_t dir, int speed){
    switch (motor){
    case M_X:
        if (dir < 2){
            output_bit(DIR_M1_IN1, dir);
            output_bit(DIR_M1_IN2, dir ^ 1);
        }else{
            output_bit(DIR_M1_IN1, 0);
            output_bit(DIR_M1_IN2, 0);
        }
        break;
    case M_Y:
        if (dir < 2){
            output_bit(DIR_M2_IN1, dir);
            output_bit(DIR_M2_IN2, dir ^ 1);
        }else{
            output_bit(DIR_M2_IN1, 0);
            output_bit(DIR_M2_IN2, 0);
        }

        break;

    case M_Z:
        if (dir < 2){
            output_bit(DIR_M3_IN1, dir);
            output_bit(DIR_M3_IN2, dir ^ 1);
        }else{
            output_bit(DIR_M3_IN1, 0);
            output_bit(DIR_M3_IN2, 0);
        }
        break;
    }
    pwmWrite(motor, abs(speed));
}

void gotoXY(float x, float y){
    errMotorX.err = x - curPos.x;
    errMotorY.err = y - curPos.y;

    errMotorX.sumErr += (errMotorX.err * TIME_D);
    errMotorY.sumErr += (errMotorY.err * TIME_D);

    float divX = (errMotorX.err - errMotorX.pError) / TIME_D;
    float divY = (errMotorY.err - errMotorY.pError) / TIME_D;

    errMotorX.outPID = (Kp_X * errMotorX.err) + (Ki_X * errMotorX.sumErr) + (Kd_X * divX);
    errMotorY.outPID = (Kp_Y * errMotorY.err) + (Ki_Y * errMotorY.sumErr) + (Kd_Y * divY);

    errMotorX.pError = errMotorX.err;
    errMotorY.pError = errMotorY.err;
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pwmWrite(uint8_t motor, float duty){
    duty = (duty < 1) ? 0 : duty;
    long duty_pwm = (duty / 100.0) * TIME_PERIOD_PWM;

    if (duty == 100){
        duty_pwm = duty_pwm + 1;
    }

    switch (motor){
    case M_X:
        set_pwm_duty(1, duty_pwm);
        break;
    case M_Y:
        set_pwm_duty(2, duty_pwm);
        break;
    case M_Z:
        // set_pwm_duty(3, duty_pwm);
        break;
    }
}

#endif