package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
@TeleOp(name="fftuner", group="TeleOp")
public class FeedForwardTuner extends LinearOpMode {
    Hardware hardware;
    String TAG = "feedfowardtuner";
    public void runOpMode(){
        hardware = new Hardware(hardwareMap);
        waitForStart();
        double prevDist = hardware.hub1Motors[0].motor.getCurrentPosition();
        double prevVelo = hardware.hub1Motors[0].motor.getVelocity(AngleUnit.RADIANS) *(Hardware.circumfrence/(2*Math.PI));
        FileWriter writer;
        try {
            writer = new FileWriter("//sdcard//FIRST//feedforwarddata.txt");
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        ElapsedTime time = new ElapsedTime();
        double startTime = time.milliseconds();
        double currentTime = startTime;
        double prevTime = startTime;
        while(!isStopRequested() && currentTime < startTime + 8000){
            sleep(50);
            hardware.allHubs.get(0).clearBulkCache();
             currentTime = time.milliseconds();
            double timeDiff = currentTime - startTime;
            double i = timeDiff/8000;
            if(isStopRequested()){
                hardware.sixWheelDrive.goStraight(-1);
                sleep(100);
            }
            hardware.sixWheelDrive.goStraight(i);
            telemetry.addData("power",i);
            double currentPosition = hardware.hub1Motors[0].motor.getCurrentPosition();
            double velocityTicks = (currentPosition-prevDist)/((currentTime-prevTime)/1000);
            double velocity = velocityTicks * Hardware.circumfrence / Hardware.ticks_per_rotation;
            prevDist = currentPosition;
            double accel = (velocity-prevVelo)/((currentTime-prevTime)/1000);
            prevTime = currentTime;
            prevVelo = velocity;
            RobotLog.dd(TAG, "Velo: " + velocity + ", Power: " + i + ", Accel: "+ accel);
            try {
                writer.write("("+ velocity + ", "+i+") , ");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
