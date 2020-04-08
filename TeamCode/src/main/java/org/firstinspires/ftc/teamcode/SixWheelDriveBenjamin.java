package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SixWheelDriveBenjamin {
    Motor Left;
    Motor Right;
    public SixWheelDriveBenjamin (Motor Left, Motor Right){
        this.Left = Left;
        this.Right = Right;
    }
    public void setPowers(double stickAngle, double power, double turnMagnitude){
        /*Write This Class. StickAngle is the joystick angle. Angle zero is facing up and you go
        counterclockwise from there.
        Power is the translational power (front/back and right/left)
        turnMagnitude is the turning power
         */
    }
}
