#ifndef POSITION_H
#define POSITION_H

BOOLEAN X_HOME = FALSE;
BOOLEAN Y_HOME = FALSE;
BOOLEAN Z_HOME = FALSE;

inline void servoControl()
{
    gripperRotate.error = gripperRotate.setAngle - gripperRotate.curAngle;
    gripperKeep.error = gripperKeep.setAngle - gripperKeep.curAngle;

    gripperRotate.curAngle += (gripperRotate.error * 0.5);
    gripperKeep.curAngle += (gripperKeep.error * 0.5);

    gripperRotate.error = (abs(gripperRotate.error) > 2) ? gripperRotate.error : 0;
    servoRotate(gripperRotate.curAngle);

    gripperKeep.error = (abs(gripperKeep.error) > 2) ? gripperKeep.error : 0;
    servoKeep(gripperKeep.curAngle);
}

inline void motorControl()
{
    uint8_t dirX = (errMotorX.err > 0) ? GO_FORWARD : GO_BACKWARD;
    uint8_t dirY = (errMotorY.err > 0) ? GO_FORWARD : GO_BACKWARD;

    if (abs(errMotorX.err) > 2)
    {
        errMotorX.outPID = (errMotorX.outPID > 100) ? 100 : map(errMotorX.outPID, 0, 100, 50, 100);
        firstFeedBack.xPosition = FALSE;
    }
    else
    {
        if (feedBack.xPosition && !firstFeedBack.xPosition)
        {
            feedBack.xPosition = FALSE;
            sentData(TASK_COMPLETE, X_POSITION);
        }
        dirX = GO_BREAK;
        errMotorX.outPID = 0;
    }

    if (abs(errMotorY.err) > 2)
    {
        errMotorY.outPID = (errMotorY.outPID > 100) ? 100 : map(errMotorY.outPID, 0, 100, 30, 100);
        firstFeedBack.yPosition = FALSE;
    }
    else
    {
        if (feedBack.yPosition && !firstFeedBack.yPosition)
        {
            feedBack.yPosition = FALSE;
            sentData(TASK_COMPLETE, Y_POSITION);
        }
        dirY = GO_BREAK;
        errMotorY.outPID = 0;
    }

    driveMotor(M_X, dirX, errMotorX.outPID);
    driveMotor(M_Y, dirY, errMotorY.outPID);
}

inline void gotoHome()
{
    if (X_HOME == FALSE)
    {
        driveMotor(M_X, GO_BACKWARD, 60);
        pulseX = 0;
    }
    else
    {
        driveMotor(M_X, GO_BREAK, 0);
    }

    if (Y_HOME == FALSE)
    {
        driveMotor(M_Y, GO_BACKWARD, 60);
        pulseY = 0;
    }
    else
    {
        driveMotor(M_Y, GO_BREAK, 0);
    }

    if (Z_HOME == FALSE)
    {
        driveMotor(M_Z, GO_BACKWARD, 100);
    }
    else
    {
        driveMotor(M_Z, GO_BREAK, 0);
        curPos.z = 0;
    }

    if (Z_HOME && Y_HOME && X_HOME)
    {
        if (feedBack.goHome)
        {
            feedBack.goHome = FALSE;
            sentData(TASK_COMPLETE, GO_HOME);
        }
    }
}

#endif