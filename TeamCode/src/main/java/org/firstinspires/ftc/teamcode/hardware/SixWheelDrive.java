package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class SixWheelDrive {
    public Motor LF;
    public Motor LB;
    public Motor RF;
    public Motor RB;
    public VelocityPID left;
    public VelocityPID right;
    public static double kP=0.018;
    public static double kD=0;
    public static double kV=0.174;
    public static double kStatic=1.282;
    public static double kA = 0.0325;
    public static double kI = 0.016;
    public static double kVAngularVelo = 1.4;
    public static double kAAngularAccel=0.15;
    public static double kStaticAngularVelo = 0;
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
        left = new VelocityPID(kP, kI, kD, kV, kStatic,kA,kVAngularVelo, kStaticAngularVelo,kAAngularAccel,time, "//sdcard//FIRST//LeftVeloPIDData.txt");
        right = new VelocityPID(kP, kI, kD, kV, kStatic,kA,kVAngularVelo,kStaticAngularVelo ,kAAngularAccel,time,"//sdcard//FIRST//RightVeloPIDData.txt");
    }
    public void goStraight(double power){
        LF.motor.setPower(power);
        LB.motor.setPower(power);
        RF.motor.setPower(power);
        RB.motor.setPower(power);
    }
    public void setMotion(double velocity, double accel, double angularVelocity,double angularAccel){
        double rightVelocity = velocity + Hardware.trackWidth/2 * angularVelocity;
        double leftVelocity = velocity - Hardware.trackWidth/2 * angularVelocity;
        left.setVelocity(leftVelocity,accel,-angularVelocity,-angularAccel);
        right.setVelocity(rightVelocity,accel,angularVelocity,angularAccel);
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
    public void setPowersNonTank(double movementY, double turnSpeed){
        double left = movementY - turnSpeed;
        double right = movementY + turnSpeed;
        double max = Math.abs(left);
        if(Math.abs(right)< max){
            max = Math.abs(right);
        }
        if(max > 1){
            left = left/max;
            right = right/max;
        }
        RobotLog.dd("sixWheelDriveDebug", "Left Power: "+left+", right power: "+right);
        LF.setPower(left);
        LB.setPower(left);
        RF.setPower(right);
        RB.setPower(right);
    }
}
