package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Multithreads.MoveArmDownAfterDropping1stWobbler;
import org.firstinspires.ftc.teamcode.Ramsete.Path;
import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;
@Autonomous(name = "RedAuto", group = "Autonomous")
public class UltimateGoalRedAuto extends AutoMethods {
    int stack = 1;
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        HardwareThreadInterface hardwareThreadInterface= new HardwareThreadInterface(hardware, this);
        PathEngine dropWobbler1;
        if(stack==0) {
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack0.txt", hardware, this);
        }
        else if(stack == 1){
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack1.txt", hardware, this);
        }
        else{
            dropWobbler1=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler1Stack2.txt", hardware, this);
        }
        PathEngine collect2ndWobbler;
        if(stack==0) {
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack0.txt", hardware, this);
        }
        else if(stack == 1){
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack1.txt", hardware, this);
        }else{
            collect2ndWobbler=new PathEngine(40, 5, "//sdcard//FIRST//UGauto//collect2ndWobblerStack4.txt", hardware, this);
        }

        PathEngine dropWobbler2;
        if(stack==0) {
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack0.txt", hardware, this);
        }else if(stack==1){
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack1.txt", hardware, this);
        }else{
            dropWobbler2 = new PathEngine(40, 5, "//sdcard//FIRST//UGauto//dropWobbler2Stack2.txt", hardware, this);
        }
        /*
        PathEngine park;
        if(stack == 0){
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack0.txt",hardware,this);
        }else if(stack == 1){
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack1.txt",hardware,this);
        }else{
            park = new PathEngine(40,5,"//sdcard//FIRST//UGauto//parkStack2.txt",hardware,this);
        }*/
        hardware.wobbler.goToWobbleStartingPos();
        hardware.wobbler.gripWobble();
        hardware.loop();
        collect2ndWobbler.init();
        dropWobbler1.init();
        dropWobbler2.init();
        waitForStart();
        hardware.updatePID = true;
        hardwareThreadInterface.start();
        dropWobbler1.run(hardware.time,20,0.7,false);
        hardware.wobbler.moveArmToGrabPos();
        sleep(750);
        hardware.wobbler.releaseWobble();
        sleep(500);
        hardware.wobbler.goToWobbleStartingPos();
        if(stack==1){
            turnTo(-90,1500,hardware);
        }
        MoveArmDownAfterDropping1stWobbler moveArmDownAfterDropping1stWobbler = new MoveArmDownAfterDropping1stWobbler(hardware,this);
        moveArmDownAfterDropping1stWobbler.start();
        collect2ndWobbler.run(hardware.time,20,0.7,false);
        sleep(750);
        hardware.wobbler.gripWobble();
        sleep(500);
        hardware.wobbler.raiseWobble();
        sleep(500);
        turnTo(-360,1000,hardware);
        dropWobbler2.run(hardware.time,20,0.7,false);
    }
}
