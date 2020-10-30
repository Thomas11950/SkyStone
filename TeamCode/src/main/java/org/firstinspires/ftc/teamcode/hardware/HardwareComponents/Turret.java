package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.FakePID;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public class Turret {
    public static double ticks_per_radian=15833.3703586*34/45;
    Hardware hardware;
    ContRotServo[] turretServos;
    private double startTurretPosition;
    public FakePID turretPID;
    public Motor encoder;
    public double CENTER_TO_TURRET_INCHES;
    public boolean updatePID;
    public double maxCounterClockwise;
    public double maxClockwise;
    public Turret(ContRotServo[] turretServos, Motor encoder, Hardware hardware){
        this.turretServos = turretServos;
        this.hardware = hardware;
        this.encoder = encoder;
        encoder.readRequested = true;
        startTurretPosition = localTurretAngleRadians();
        //turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
        turretPID = new FakePID(1,0.0349066,hardware.time);
        updatePID = false;
    }
    public void setTurretAngle(double globalTurretAngle){//global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double desiredLocalTurretAngle = MathFunctions.keepAngleWithinSetRange(-maxClockwise,maxCounterClockwise,globalTurretAngle - hardware.angle);
        turretPID.setState(desiredLocalTurretAngle);
    }
    public void updateTurretPID(){
        double output = turretPID.updateCurrentStateAndGetOutput(localTurretAngleRadians() - startTurretPosition);
        setAllTurretServoPowers(output);
    }
    public double[] getTurretPosition(){
        return MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),CENTER_TO_TURRET_INCHES,hardware.angle);
    }
    public void setAllTurretServoPowers(double power){
        for(ContRotServo crservo: turretServos){
            crservo.setPower(power);
        }
    }
    public void pointTowardsHighGoal(){
        double[] currentPoint = getTurretPosition();
        double angleToPointTo = Math.atan2((currentPoint[1]- FieldConstants.highGoalPosition[1]),(currentPoint[0]-FieldConstants.highGoalPosition[0]));
        setTurretAngle(angleToPointTo);
    }
    public double localTurretAngleRadians(){
        return -encoder.getCurrentPosition()/ticks_per_radian;
    }
}
