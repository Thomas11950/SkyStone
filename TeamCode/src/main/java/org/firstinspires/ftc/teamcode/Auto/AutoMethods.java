package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public abstract class AutoMethods extends LinearOpMode {
    public void turnTo(double targetAngleRadians,double duration,Hardware hardware){
        TurretPID headingPID = new TurretPID(1.2,6,0.12,Math.toRadians(20), hardware.time);
        headingPID.setState(Math.toRadians(targetAngleRadians));
        double startTime = hardware.time.milliseconds();
        while(hardware.time.milliseconds()-startTime<duration){
            double output = headingPID.updateCurrentStateAndGetOutput(hardware.angle);
            hardware.sixWheelDrive.turn(output);
            hardware.loop();
        }
    }
}
