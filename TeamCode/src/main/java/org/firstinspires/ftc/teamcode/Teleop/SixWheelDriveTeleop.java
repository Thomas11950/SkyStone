package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SixWheelDrive;

@TeleOp(name="6wd", group="TeleOp")
public class SixWheelDriveTeleop extends OpMode {
    Hardware hardware;
    public void init(){
        hardware = new Hardware(hardwareMap);
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.55;
    }
    public void loop(){
        double leftAbsValue = Math.abs(gamepad1.left_stick_y);
        double rightAbsValue = Math.abs(gamepad1.right_stick_y);
        double leftPower = logistic(leftAbsValue, 1,7.2) * -gamepad2.left_stick_y/leftAbsValue;
        double rightPower = logistic(rightAbsValue,1,7.2) * -gamepad1.right_stick_y/rightAbsValue;
        hardware.sixWheelDrive.LF.motor.setPower(leftPower);
        hardware.sixWheelDrive.LB.motor.setPower(leftPower);
        hardware.sixWheelDrive.RF.motor.setPower(rightPower);
        hardware.sixWheelDrive.RB.motor.setPower(rightPower);
        hardware.loop();
        telemetry.addLine("left position: " + hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + -hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+hardware.banglePrev + ", angle 2: "+hardware.danglePrev);
        telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
        telemetry.addLine("XAlt: " + hardware.xPosTicksAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt: "+hardware.yPosTicksAlt*Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("XAltAlt: "+ hardware.xPosTicksAltAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAltAlt: " + hardware.yPosTicksAltAlt * Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("angularVeloTracker: "+hardware.integratedAngularVeloTracker);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
    }
}
