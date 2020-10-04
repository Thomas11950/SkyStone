package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Turret;

public class TurretPIDTuner extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        while(!isStopRequested()){
            if(gamepad1.left_bumper){
                hardware.turret.turretPID.kP+=0.1;
            }
            hardware.turret.setTurretAngle(Math.toRadians(90));
            hardware.loop();
        }
    }
}
