package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterPID extends VelocityPID {
    public double disableIntegralThreshold;
    public ShooterPID(double kP, double kI, double kD, double kV, double kStatic, double kA, double disableIntegralThreshold, ElapsedTime time, String outputFileName) {
        super(kP, kI, kD, kV, kStatic, kA, time, outputFileName);
        this.disableIntegralThreshold = disableIntegralThreshold;
    }
    @Override
    public boolean shouldIntegralBeZeroed(double error){
        return Math.abs(error)>disableIntegralThreshold;
    }
}
