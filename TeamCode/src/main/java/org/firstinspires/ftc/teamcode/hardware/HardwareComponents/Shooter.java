package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
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
    public boolean firstUpdateShooterPIDFLoop = true;
    private double prevShooterPos;
    public boolean updatePID;
    public double rampPostion = 0;
    AutoShootInfo info;
    public Shooter(Motor shooterMotor1, Motor shooterMotor2, RegServo shootAngleController, Hardware hardware){
        this.shootAngleController = shootAngleController;
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        shooterMotor2.readRequested = true;
        this.shooterMotor1.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.shooterMotor2.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.hardware = hardware;
        shooterVeloPID = new ShooterPID(0.0025,0.001,0,0.005,2.84,0,150,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        shooterVeloPID.integralAntiWindupActive = false;
        updatePID = false;
        info = new AutoShootInfo();
    }
    public void updateShooterPIDF(double deltaTime){
        if(firstUpdateShooterPIDFLoop){
            prevShooterPos = shooterMotor2.getCurrentPosition();
            firstUpdateShooterPIDFLoop = false;
        }
        double shooterPos = shooterMotor2.getCurrentPosition();
        double currentVelo = (shooterPos - prevShooterPos)/deltaTime;
        Hardware.telemetry.addData("shooter velo",currentVelo);
        Hardware.telemetry.addData("shooterPIDsetstate", shooterVeloPID.desiredState);
        prevShooterPos = shooterPos;
        double outputPower = shooterVeloPID.updateCurrentStateAndGetOutput(currentVelo);
        double voltage = VelocityPIDDrivetrain.getBatteryVoltage();
        Hardware.telemetry.addData("outputVoltage",outputPower);
        shooterMotor1.setPower(outputPower/voltage);
        shooterMotor2.setPower(outputPower/voltage);
    }
    public void setRampPosition(double position){
        rampPostion = position;
        shootAngleController.setPosition(START_TICKS_RAMP - rampPostion *(START_TICKS_RAMP-END_TICKS_RAMP));
    }
    public void autoRampPositionForHighGoal(double distanceToGoal){
        double rampAngle = 0;
        for(int i = 0; i < info.distances.size()-1; i++){
            if(MathFunctions.isInBetween(info.distances.get(i), info.distances.get(i+1), distanceToGoal)){
                Hardware.telemetry.addData("index of point",i);
                double slope = (info.rampAngles.get(i+1) - info.rampAngles.get(i))/(info.distances.get(i+1)-info.distances.get(i));
                rampAngle = slope*(distanceToGoal - info.distances.get(i))+info.rampAngles.get(i);
            }
        }
        Hardware.telemetry.addData("distToGoal",distanceToGoal);
        Hardware.telemetry.addData("rampAngle",rampAngle);
        setRampPosition(rampAngle);
    }
}
