package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityPID {
    double kP;
    double kI;
    double kD;
    public double kV;
    double kStatic;
    double kA;
    double integral;
    public double targetVelocity;
    ElapsedTime time;
    double prevTime;
    double prevError;
    boolean firstUpdatePowerLoop;
    public VelocityPID(double kP, double kI, double kD, double kV, double kStatic, double kA, ElapsedTime time){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kStatic = kStatic;
        this.kA = kA;
        integral = 0;
        targetVelocity = 0;
        this.time = time;
        firstUpdatePowerLoop = true;
    }
    public void clearI(){
        integral = 0;
    }
    public synchronized void setVelocity(double targetVelocity){
        firstUpdatePowerLoop = false;
        this.targetVelocity = targetVelocity;
    }
    public synchronized double updatePower(double currentVelocity){
        double currentTime = time.milliseconds();
        double deltaTime = (currentTime - prevTime)/1000;
        prevTime = currentTime;
        double error = targetVelocity - currentVelocity;
        double deltaError = error-prevError;
        prevError = error;
        integral+=error*deltaTime;
        double derivative = deltaError / deltaTime;
        if(targetVelocity < 0){

            return error * kP + integral * kI + derivative * kD - kStatic + targetVelocity * kV;
        }
        else{

            return error * kP + integral * kI + derivative * kD + kStatic + targetVelocity * kV;
        }
    }
}
