package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.SixWheelDrive;
import org.firstinspires.ftc.teamcode.vision.T265;

@TeleOp(name="6wd", group="TeleOp")
public class SixWheelDriveTeleop extends OpMode {
    Hardware hardware;
    boolean slowMode;
    public void init(){
        if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        hardware = new Hardware(hardwareMap);
        slowMode = false;
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
    public void start(){
        T265.slamra.start();
    }
    public void loop(){
        double leftPower;
        double rightPower;
        if(gamepad1.left_trigger > 0){
            slowMode = true;
        }
        else{
            slowMode = false;
        }
        if(!slowMode) {
            double leftAbsValue = Math.abs(gamepad1.left_stick_y);
            double rightAbsValue = Math.abs(gamepad1.right_stick_y);
            leftPower = logistic(leftAbsValue, 1, 7.2) * -gamepad1.left_stick_y / leftAbsValue;
            rightPower = logistic(rightAbsValue, 1, 7.2) * -gamepad1.right_stick_y / rightAbsValue;
        }
        else{
            leftPower = -gamepad1.left_stick_y*0.5;
            rightPower = -gamepad1.right_stick_y*0.5;
        }
         hardware.sixWheelDrive.LF.motor.setPower(leftPower);
        hardware.sixWheelDrive.LB.motor.setPower(leftPower);
        hardware.sixWheelDrive.RF.motor.setPower(rightPower);
        hardware.sixWheelDrive.RB.motor.setPower(rightPower);
        hardware.loop();
        com.spartronics4915.lib.T265Camera.CameraUpdate up = T265.slamra.getLastReceivedCameraUpdate();
        telemetry.addLine("camera X: "+up.pose.getTranslation().getX()/0.0254+", camera Y: "+up.pose.getTranslation().getY()/0.0254);
        telemetry.addLine("left Power: " + leftPower + ", right Power: "+rightPower);
        telemetry.addLine("left position: " + hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + -hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+hardware.banglePrev + ", angle 2: "+hardware.danglePrev);
        telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
        telemetry.addLine("XAlt: " + hardware.xPosTicksAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt: "+hardware.yPosTicksAlt*Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("XAltAlt: "+ hardware.xPosTicksAltAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAltAlt: " + hardware.yPosTicksAltAlt * Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("angularVeloTracker: "+hardware.integratedAngularVeloTracker);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
    }
    public void stop(){
        T265.slamra.stop();
    }
}
