package org.firstinspires.ftc.teamcode.hardware;

public class SixWheelDrive {
    Motor LF;
    Motor LB;
    Motor RF;
    Motor RB;
    public SixWheelDrive (Motor LF, Motor LB, Motor RF, Motor RB){
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
    }
    public void setPowers(double velocity, double angularVelocity){
        double rightVelocity = velocity + Hardware.odoWidth/2 * angularVelocity;
        double leftVelocity = velocity - Hardware.odoWidth/2 * angularVelocity;
        LF.setVelocity(leftVelocity);
        LB.setVelocity(leftVelocity);
        RF.setVelocity(rightVelocity);
        RB.setVelocity(rightVelocity);
    }
}
