package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.FakePID;
import org.firstinspires.ftc.teamcode.hardware.PID.GenericPID;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public class Turret {
    public static double ticks_per_radian=15833.3703586*34/45/2*188/180;
    Hardware hardware;
    ContRotServo[] turretServos;
    private double startTurretPosition;
    public PIDwithBasePower turretPID;
    public Motor encoder;
    public double CENTER_TO_TURRET_INCHES;
    public boolean updatePID;
    public double maxCounterClockwise=180;
    public double maxClockwise=180;
    public Turret(ContRotServo[] turretServos, Motor encoder, Hardware hardware){
        this.turretServos = turretServos;
        this.hardware = hardware;
        this.encoder = encoder;
        encoder.readRequested = true;
        startTurretPosition = localTurretAngleRadians();
        //turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
        turretPID = new PIDwithBasePower(0.7/Math.toRadians(40),0.3,0.3,0.2,Math.toRadians(3),Math.toRadians(20), hardware.time);
        updatePID = false;
    }
    public double getTurretOffset(double distanceToGoal){
        return distanceToGoal * -0.00126 + 0.306;
    }
    public void setTurretAngle(double globalTurretAngle){//global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double desiredLocalTurretAngle = MathFunctions.keepAngleWithin180Degrees(globalTurretAngle - hardware.angle);
        if(desiredLocalTurretAngle > 100){
            desiredLocalTurretAngle = 100;
        }
        Hardware.telemetry.addData("desiredLocalTurretAngle",desiredLocalTurretAngle);
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
