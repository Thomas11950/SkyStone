package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
@TeleOp(name="6wd", group="TeleOp")
public class SixWheelDriveTeleop extends OpMode {
    Hardware hardware;
    public void init(){
        hardware = new Hardware(hardwareMap);
    }
    public void loop(){
        double leftPower = -Math.pow(gamepad1.left_stick_y,3);
        double rightPower = -Math.pow(gamepad1.right_stick_y,3);
        hardware.sixWheelDrive.LF.motor.setPower(leftPower);
        hardware.sixWheelDrive.LB.motor.setPower(leftPower);
        hardware.sixWheelDrive.RF.motor.setPower(rightPower);
        hardware.sixWheelDrive.RB.motor.setPower(rightPower);
        hardware.loop();
        telemetry.addLine("left position: " + hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + -hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle));
        telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
        telemetry.addLine("XAlt: " + hardware.xPosTicksAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt: "+hardware.yPosTicksAlt*Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
    }
}
