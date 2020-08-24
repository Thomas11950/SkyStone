package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareThreadInterface extends Thread {
    public Hardware hardware;
    LinearOpMode parentOP;
    public HardwareThreadInterface(Hardware hardware, LinearOpMode parentOP){
        this.hardware = hardware;
        this.parentOP = parentOP;
    }
    public void run(){
        while(!parentOP.isStopRequested()){
            hardware.loop();
            if(isInterrupted()){
                return;
            }
        }
    }
}
