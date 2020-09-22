package org.firstinspires.ftc.teamcode.SkystoneSim;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Ramsete.Path;
import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;

@Autonomous(name = "skystoneSim", group = "Autonomous")
public class skystoneSim extends LinearOpMode {
    public void runOpMode(){
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(new Hardware(hardwareMap,telemetry), this);
        Hardware hardware = hardwareThreadInterface.hardware;
        PathEngine collect1 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//collect1.txt",hardware, this);
        collect1.init();
        PathEngine deposit1 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//deposit1.txt",hardware,this);
        deposit1.init();
        PathEngine collect2 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//collect2.txt",hardware,this);
        collect2.init();
        PathEngine deposit2 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//deposit2.txt",hardware,this);
        deposit2.init();
        PathEngine collect3 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//collect3.txt",hardware,this);
        collect3.init();
        PathEngine deposit3 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//deposit3.txt",hardware,this);
        deposit3.init();
        PathEngine collect4 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//collect4.txt",hardware,this);
        collect4.init();
        PathEngine deposit4 = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//deposit4.txt",hardware,this);
        deposit4.init();
        PathEngine park = new PathEngine(60,5,"//sdcard//FIRST//SkystoneSimAuto//park.txt",hardware,this);
        park.init();
        waitForStart();
        hardwareThreadInterface.start();
        collect1.run(hardware.time,50,0.7,true);
        deposit1.run(hardware.time,50,0.7,false);
        collect2.run(hardware.time,50,0.7,true);
        deposit2.run(hardware.time,50,0.7,false);
        collect3.run(hardware.time,50,0.7,true);
        deposit3.run(hardware.time,50,0.7,false);
        collect4.run(hardware.time,50,0.7,true);
        deposit4.run(hardware.time,50,0.7,false);
        park.run(hardware.time,50,0.7,true);
        hardware.updatePID = false;
        hardware.sixWheelDrive.LF.setPower(0);
        hardware.sixWheelDrive.LB.setPower(0);
        hardware.sixWheelDrive.RF.setPower(0);
        hardware.sixWheelDrive.RB.setPower(0);
        while(!isStopRequested()) {
            telemetry.addLine("X: " + hardware.getX() + ", Y: " + hardware.getY());
            telemetry.update();
        }
    }
}
