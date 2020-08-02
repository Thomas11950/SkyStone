package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.FileDescriptor;
import java.io.FileWriter;
import java.io.IOException;

public class VelocityPID {
    double kP;
    double kI;
    double kD;
    public double kV;
    double kStatic;
    double kA;
    double kDecel;
    double integral;
    public double targetVelocity;
    ElapsedTime time;
    double prevTime;
    double prevError;
    boolean firstUpdatePowerLoop;
    boolean firstSetVelocityLoop;
    public FileWriter writer;
    double startTime;
    double prevVelo;
    double accel;
    double prevTimeAccel;
    double kVAngularVelo;
    double targetAngularVelo;
    double kStaticAngularVelo;
    double kAAngularAccel;
    double angularAccel;
    public VelocityPID(double kP, double kI, double kD, double kV, double kStatic, double kA, double kDecel, double kVAngularVelo, double kStaticAngularVelo, double kAAngularAccel, ElapsedTime time, String outputFileName) {
        try {
            writer = new FileWriter(outputFileName);
        }
        catch(IOException e){
            return;
        }
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kStatic = kStatic;
        this.kA = kA;
        this.kDecel = kDecel;
        integral = 0;
        targetVelocity = 0;
        this.time = time;
        firstUpdatePowerLoop = true;
        firstSetVelocityLoop = true;
        prevVelo = 0;
        accel = 0;
        this.kVAngularVelo = kVAngularVelo;
        this.kStaticAngularVelo = kStaticAngularVelo;
        this.kAAngularAccel = kAAngularAccel;
    }
    public void clearI(){
        integral = 0;
    }
    public synchronized void setVelocity(double targetVelocity, double accel){
        if(firstSetVelocityLoop){
            firstSetVelocityLoop=false;
            prevTimeAccel = time.milliseconds();
        }
        this.targetVelocity = targetVelocity;
        double currentTime = time.milliseconds();
        this.accel = accel;
        prevTimeAccel = currentTime;
        prevVelo = targetVelocity;
        targetAngularVelo = 0;
    }
    public synchronized void setVelocity(double targetVelocity, double accel, double targetAngularVelo, double angularAccel){
        if(firstSetVelocityLoop){
            firstSetVelocityLoop=false;
            prevTimeAccel = time.milliseconds();
        }
        this.targetVelocity = targetVelocity;
        double currentTime = time.milliseconds();
        this.accel = accel;
        prevTimeAccel = currentTime;
        prevVelo = targetVelocity;
        this.targetAngularVelo = targetAngularVelo;
        this.angularAccel = angularAccel;
    }
    public synchronized double updatePower(double currentVelocity){
        double currentTime = time.milliseconds();
        if(firstUpdatePowerLoop){
            firstUpdatePowerLoop = false;
            prevTime = currentTime;
            startTime = prevTime;
        }
        double deltaTime = (currentTime - prevTime)/1000;
        prevTime = currentTime;
        double error = targetVelocity - currentVelocity;
        double deltaError = error-prevError;
        prevError = error;
        integral+=error*deltaTime;
        double derivative = deltaError / deltaTime;
        double batteryVoltage = getBatteryVoltage();
        RobotLog.dd("FFPID","error: "+error + ", AngularAccel: "+angularAccel);
        double toReturn;
        if(targetVelocity < 0){
            if((targetVelocity >= 0 && accel > 0) ||(targetVelocity <= 0 && accel < 0)) {
                toReturn = error * kP + integral * kI + derivative * kD - kStatic / batteryVoltage + (targetVelocity * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else if((targetVelocity < 0 && accel > 0)||(targetVelocity > 0 && accel < 0)){
                toReturn = error * kP + integral * kI + derivative * kD - kStatic / batteryVoltage + (targetVelocity * kV) / batteryVoltage + (accel * kDecel) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else{
                toReturn = error * kP + integral * kI + derivative * kD - kStatic / batteryVoltage + (targetVelocity * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
        }
        else{
            if((targetVelocity >= 0 && accel > 0) ||(targetVelocity <= 0 && accel < 0)) {
                toReturn = error * kP + integral * kI + derivative * kD + kStatic / batteryVoltage + (targetVelocity * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else if((targetVelocity < 0 && accel > 0)||(targetVelocity > 0 && accel < 0)){
                toReturn = error * kP + integral * kI + derivative * kD + kStatic / batteryVoltage + (targetVelocity * kV) / batteryVoltage + (accel * kDecel) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
            else{
                toReturn = error * kP + integral * kI + derivative * kD + kStatic / batteryVoltage + (targetVelocity * kV) / batteryVoltage + (accel * kA) / batteryVoltage + kVAngularVelo * targetAngularVelo / batteryVoltage + kAAngularAccel * angularAccel / batteryVoltage;
            }
        }
        try {
            writer.write("Time: " + (currentTime-startTime)/1000+", RequestedV: " + targetVelocity + ", Velo: " + currentVelocity+", Power: "+toReturn+ ", "+ "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
        return toReturn;
    }
    public static double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : Hardware.getHWmap().voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
