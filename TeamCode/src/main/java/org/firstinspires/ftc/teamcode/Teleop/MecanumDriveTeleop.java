package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.T265;

@TeleOp(name="mec", group="TeleOp")
public class MecanumDriveTeleop extends OpMode {
    HardwareMecanum hardware;
    public void init(){
        if(hardwareMap==null){

            telemetry.addLine("hwmap  null");
            telemetry.update();
        }
        else{
            telemetry.addLine("hwmap not null");
            telemetry.update();

        }

        if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        hardware = new HardwareMecanum(hardwareMap,telemetry);

    }
    public void loop(){
        hardware.mecanumDrive.setPowers(-gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x);
        hardware.loop();

        com.spartronics4915.lib.T265Camera.CameraUpdate up = T265.slamra.getLastReceivedCameraUpdate();
        telemetry.addLine("camera X: "+up.pose.getTranslation().getX()/0.0254+", camera Y: "+up.pose.getTranslation().getY()/0.0254);
        telemetry.addLine("left position: " + hardware.previousPortReading + ", right position: " + hardware.previousStarboardReading + ", lateral position: " + hardware.previousLateralReading);
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+Math.toDegrees(hardware.angle1) + ", angle 2: "+Math.toDegrees(hardware.angle2));
        telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
        telemetry.addLine("XAlt: " + hardware.xPosTicksAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt: "+hardware.yPosTicksAlt*Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("XAlt2: " + hardware.xPosTicksAlt2 * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAlt2: "+hardware.yPosTicksAlt2*Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("XAltAlt: "+ hardware.xPosTicksAltAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAltAlt: " + hardware.yPosTicksAltAlt * Hardware.circumfrence/Hardware.ticks_per_rotation);
        telemetry.addLine("angularVeloTracker: "+hardware.integratedAngularVeloTracker);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
    }
    public void stop(){
        T265.slamra.stop();
    }
}
