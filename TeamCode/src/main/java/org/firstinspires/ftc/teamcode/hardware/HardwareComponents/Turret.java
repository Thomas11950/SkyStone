package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public class Turret {
    public static double ticks_per_radian;
    Hardware hardware;
    CRServo[] turretServos;
    private double startTurretPosition;
    public TurretPID turretPID;
    public Motor encoder;
    public double CENTER_TO_TURRET_INCHES;
    public Turret(CRServo[] turretServos, Motor encoder, Hardware hardware){
        this.turretServos = turretServos;
        this.hardware = hardware;
        this.encoder = encoder;
        startTurretPosition = encoder.getCurrentPosition();
        turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
    }
    public void setTurretAngle(double globalTurretAngle){//global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double desiredLocalTurretAngle = MathFunctions.keepAngleWithin180Degrees(globalTurretAngle - hardware.angle);
        turretPID.setState(desiredLocalTurretAngle);
    }
    public void updateTurretPID(){
        double output = turretPID.updateCurrentStateAndGetOutput(encoder.getCurrentPosition()/ticks_per_radian - startTurretPosition);
        for(CRServo crservo: turretServos){
            crservo.setPower(output);
        }
    }
    public double[] getTurretPosition(){
        return MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),CENTER_TO_TURRET_INCHES,hardware.angle);
    }
}
