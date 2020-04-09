package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareThreadInterface extends Thread {
    public Hardware hardware;
    LinearOpMode parentOP;
    public HardwareThreadInterface(HardwareMap hardwareMap){
        this.hardware = new Hardware(hardwareMap);
    }
    public void run(){
        while(!parentOP.isStarted()){
            hardware.loop();
        }
    }
}
