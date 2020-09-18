package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.RegServo;
import org.firstinspires.ftc.teamcode.hardware.PID.VelocityPID;

public class Shooter {
    public Motor shooterMotor;
    public VelocityPID shooterVeloPID;
    Hardware hardware;
    public RegServo shootAngleController;
    public Shooter(Motor shooterMotor, Hardware hardware){
        this.shooterMotor = shooterMotor;
        this.hardware = hardware;
        shooterVeloPID = new VelocityPID(1,1,1,1,1,1,hardware.time,"");
    }
    public void updateShooterPIDF(double currentVelo){
        shooterMotor.setPower(shooterVeloPID.updateCurrentStateAndGetOutput(currentVelo));
    }
}
