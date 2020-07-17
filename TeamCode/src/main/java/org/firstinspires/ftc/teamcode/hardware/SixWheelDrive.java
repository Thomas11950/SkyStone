package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SixWheelDrive {
    public Motor LF;
    public Motor LB;
    public Motor RF;
    public Motor RB;
    public VelocityPID left;
    public VelocityPID right;
    public static double kP=0.017;
    public static double kD=0.00014;
    public static double kV=0.0129;
    public static double kStatic=0.05363;
    public static double kA = 0;
    public static double kI = 0.025;
    public SixWheelDrive (Motor LF, Motor LB, Motor RF, Motor RB, ElapsedTime time){
        this.LF = LF;
        this.LB = LB;
        this.RF = RF;
        this.RB = RB;
        LF.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LB.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RF.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RB.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LF.readRequested = true;
        RF.readRequested = true;
        left = new VelocityPID(kP, kI, kD, kV, kStatic,kA, time);
        right = new VelocityPID(kP, kI, kD, kV, kStatic,kA, time);
    }
    public void goStraight(double power){
        LF.motor.setPower(power);
        LB.motor.setPower(power);
        RF.motor.setPower(power);
        RB.motor.setPower(power);
    }
    public void setMotion(double velocity, double angularVelocity){
        double rightVelocity = velocity + Hardware.trackWidth/2 * angularVelocity;
        double leftVelocity = velocity - Hardware.trackWidth/2 * angularVelocity;
        left.setVelocity(leftVelocity);
        right.setVelocity(rightVelocity);
    }
    public void updatePID(double leftVelocityFromOdo, double rightVelocityFromOdo){
        double leftPower = left.updatePower(leftVelocityFromOdo);
        double rightPower = right.updatePower(rightVelocityFromOdo);
        if(leftPower > 1){
            leftPower = 1;
        }
        if(leftPower < -1){
            leftPower = -1;
        }
        if(rightPower > 1){
            rightPower = 1;
        }
        if(rightPower < -1){
            rightPower = -1;
        }
        RF.setPower(rightPower);
        RB.setPower(rightPower);
        LF.setPower(leftPower);
        LB.setPower(leftPower);
    }
    public void turn(double power){
        RF.motor.setPower(power);
        RB.motor.setPower(power);
        LF.motor.setPower(-power);
        LB.motor.setPower(-power);
    }
}
