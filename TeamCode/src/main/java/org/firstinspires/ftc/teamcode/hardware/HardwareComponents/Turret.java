package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public class Turret {
    public static double ticks_per_radian;
    Hardware hardware;
    ContRotServo[] turretServos;
    private double startTurretPosition;
    public TurretPID turretPID;
    public Motor encoder;
    public double CENTER_TO_TURRET_INCHES;
    public boolean updatePID;
    public Turret(ContRotServo[] turretServos, Motor encoder, Hardware hardware){
        this.turretServos = turretServos;
        turretServos[0] = new ContRotServo(hardware.hardwareMap.get(CRServo.class,"turretServo1"));
        turretServos[1] = new ContRotServo(hardware.hardwareMap.get(CRServo.class,"turretServo2"));
        this.hardware = hardware;
        this.encoder = encoder;
        startTurretPosition = encoder.getCurrentPosition();
        turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
        updatePID = false;
    }
    public void setTurretAngle(double globalTurretAngle){//global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double desiredLocalTurretAngle = MathFunctions.keepAngleWithin180Degrees(globalTurretAngle - hardware.angle);
        turretPID.setState(desiredLocalTurretAngle);
    }
    public void updateTurretPID(){
        double output = turretPID.updateCurrentStateAndGetOutput(encoder.getCurrentPosition()/ticks_per_radian - startTurretPosition);
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
}
