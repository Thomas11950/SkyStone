package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretPID extends GenericPID {
    public double disableIntegralThreshold;
    public boolean errorPositivePreviously;
    public boolean firstShouldIntegralBeZeroedLoop;
    public boolean loopTurretPID;
    public TurretPID(double kP, double kI, double kD, double disableIntegralThreshold, ElapsedTime time) {
        super(kP, kI, kD, time);
        this.disableIntegralThreshold = disableIntegralThreshold;
        this.integralAntiWindupActive = true;
        firstShouldIntegralBeZeroedLoop = true;
        loopTurretPID = false;
    }

    @Override
    public boolean shouldIntegralBeZeroed(double error) {
        if(firstShouldIntegralBeZeroedLoop){
            if(error>=0){
                errorPositivePreviously = true;
            }
            else if(error < 0){
                errorPositivePreviously = false;
            }
            firstShouldIntegralBeZeroedLoop = false;
        }
        if(Math.abs(disableIntegralThreshold) > Math.abs(error)){
            return true;
        }
        else if(((errorPositivePreviously && error < 0) || (!errorPositivePreviously && error > 0)) || error == 0){
            return true;
        }
        else{
            return false;
        }
    }
}
