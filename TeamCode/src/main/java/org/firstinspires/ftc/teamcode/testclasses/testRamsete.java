package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class testRamsete extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap);
        PathEngine ramesetePath = new PathEngine(40,6.57,"//sdcard//FIRST//points.txt.txt",hardware);
        ramesetePath.init();
        waitForStart();
        ramesetePath.run(hardware.time,1,1);
    }
}
