package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive {
    Motor LF;
    Motor LB;
    Motor RF;
    Motor RB;
    public MecanumDrive (Motor LF, Motor LB, Motor RF, Motor RB){
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
    }
    public void setPowers(double stickAngle, double power, double turnMagnitude){
        double movementY = power*Math.cos(stickAngle);
        double movementX = power*Math.sin(stickAngle);
        double LFpower = movementY-turnMagnitude-movementX;
        double LBpower = movementY-turnMagnitude+movementX;
        double RFpower = movementY+turnMagnitude+movementX;
        double RBpower = movementY+turnMagnitude-movementX;
        double largestPower = LFpower;
        if(LBpower > LFpower)
            largestPower = LBpower;
        if(RFpower > LBpower)
            largestPower = RFpower;
        if(RBpower > RFpower){
            largestPower = RBpower;
        }
        LFpower = LFpower / largestPower;
        LBpower = LBpower / largestPower;
        RFpower = RFpower / largestPower;
        RBpower = RBpower / largestPower;
        LF.setPower(LFpower);
        LB.setPower(LBpower);
        RF.setPower(RFpower);
        RB.setPower(RBpower);
    }
}
