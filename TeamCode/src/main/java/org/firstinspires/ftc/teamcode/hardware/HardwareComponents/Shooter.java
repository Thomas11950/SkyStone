package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.RegServo;
import org.firstinspires.ftc.teamcode.hardware.PID.VelocityPID;

public class Shooter {
    public final static double ZERO_DEGREES_TICKS = 0.064;
    public Motor shooterMotor1;
    public Motor shooterMotor2;
    public VelocityPID shooterVeloPID;
    Hardware hardware;
    public RegServo shootAngleController;
    private boolean firstUpdateShooterPIDFLoop = true;
    private double prevShooterPos;
    public boolean updatePID;
    public Shooter(Motor shooterMotor1, Motor shooterMotor2, RegServo shootAngleController, Hardware hardware){
        this.shootAngleController = shootAngleController;
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        this.shooterMotor1 = new Motor(hardware.hardwareMap.get(DcMotorEx.class,"shooterMotor1"));
        this.shooterMotor2 = new Motor(hardware.hardwareMap.get(DcMotorEx.class,"shooterMotor2"));
        this.shootAngleController = new RegServo(hardware.hardwareMap.get(Servo.class,"shootAngleController"));
        this.shooterMotor1.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.shooterMotor2.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.hardware = hardware;
        shooterVeloPID = new VelocityPID(0.01,0.01,0.5,0.0059,3.12,0,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        updatePID = false;
    }
    public void updateShooterPIDF(double deltaTime){
        if(firstUpdateShooterPIDFLoop){
            prevShooterPos = shooterMotor1.getCurrentPosition();
        }
        double shooterPos = shooterMotor1.getCurrentPosition();
        double currentVelo = (shooterPos - prevShooterPos)/deltaTime;
        prevShooterPos = shooterPos;
        double outputPower = shooterVeloPID.updateCurrentStateAndGetOutput(currentVelo);
        shooterMotor1.setPower(outputPower);
        shooterMotor2.setPower(outputPower);
    }
    public void setRampDegrees(double degrees){
        shootAngleController.setPosition(degrees/270+ZERO_DEGREES_TICKS);
    }
}
