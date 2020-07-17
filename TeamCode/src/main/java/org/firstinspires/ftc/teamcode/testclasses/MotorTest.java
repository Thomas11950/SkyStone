package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
@TeleOp(name="MotorTest", group="TeleOp")
public class MotorTest extends OpMode {
    Hardware hardware;
    public void init(){
        hardware = new Hardware(hardwareMap);
    }
    public void loop(){
        hardware.sixWheelDrive.LB.motor.setPower(0.3);
    }
}
