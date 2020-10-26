package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.VelocityPIDDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.RegServo;
import org.firstinspires.ftc.teamcode.hardware.PID.VelocityPID;

public class Shooter {
    public final static double START_TICKS_RAMP = 1;
    public final static double END_TICKS_RAMP = 0.9238;
    public Motor shooterMotor1;
    public Motor shooterMotor2;
    public VelocityPID shooterVeloPID;
    Hardware hardware;
    public RegServo shootAngleController;
    private boolean firstUpdateShooterPIDFLoop = true;
    private double prevShooterPos;
    public boolean updatePID;
    public double rampPostion = START_TICKS_RAMP;
    public Shooter(Motor shooterMotor1, Motor shooterMotor2, RegServo shootAngleController, Hardware hardware){
        this.shootAngleController = shootAngleController;
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        shooterMotor2.readRequested = true;
        this.shooterMotor1.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.shooterMotor2.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.hardware = hardware;
        shooterVeloPID = new VelocityPID(0.0025,0.001,0,0.005,2.84,0,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        shooterVeloPID.integralAntiWindupActive = false;
        updatePID = false;
    }
    public void updateShooterPIDF(double deltaTime){
        if(firstUpdateShooterPIDFLoop){
            prevShooterPos = shooterMotor2.getCurrentPosition();
            firstUpdateShooterPIDFLoop = false;
        }
        double shooterPos = shooterMotor2.getCurrentPosition();
        double currentVelo = (shooterPos - prevShooterPos)/deltaTime;
        Hardware.telemetry.addData("shooter velo",currentVelo);
        prevShooterPos = shooterPos;
        double outputPower = shooterVeloPID.updateCurrentStateAndGetOutput(currentVelo);
        double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
        shooterMotor1.setPower(outputPower/voltage);
        shooterMotor2.setPower(outputPower/voltage);
    }
    public void setRampPosition(double position){
        rampPostion = position;
        shootAngleController.setPosition(START_TICKS_RAMP - rampPostion *(START_TICKS_RAMP-END_TICKS_RAMP));
    }
    public void autoRampPositionForHighGoal(double distanceToGoal){

    }
}
