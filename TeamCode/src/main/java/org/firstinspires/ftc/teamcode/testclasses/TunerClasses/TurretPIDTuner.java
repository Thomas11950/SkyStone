package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Turret;

public class TurretPIDTuner extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);
        Turret turret = new Turret(new CRServo[2], hardware.hub1Motors[3],hardware);//NEED TO REDO
        while(!isStopRequested()){
            if(gamepad1.left_bumper){
                turret.turretPID.kP+=0.1;
            }
            turret.setTurretAngle(Math.toRadians(90));
            hardware.loop();
        }
    }
}
