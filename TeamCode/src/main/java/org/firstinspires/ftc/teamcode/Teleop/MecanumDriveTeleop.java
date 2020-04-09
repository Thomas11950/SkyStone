package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class MecanumDriveTeleop extends OpMode {
    Hardware hardware;
    public void init(){
        hardware = new Hardware(hardwareMap);
    }
    public void loop(){
        hardware.mecanumDrive.setPowersTeleop(-gamepad1.left_stick_x,gamepad1.left_stick_y,-gamepad1.right_stick_x);
    }
}
