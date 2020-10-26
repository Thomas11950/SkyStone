package org.firstinspires.ftc.teamcode.hardware.PID;

import com.qualcomm.robotcore.util.ElapsedTime;
//I make this extend GenericPID so I could use it like a PID loop, but in truth it isn't. This class literally just returns a set power until the setpoint is reached.
//kP is that set power
public class FakePID extends GenericPID {
    double leewayDistance;
    public FakePID(double kP, double leewayDistance,  ElapsedTime time) {
        super(kP, 0, 0, time);
        this.leewayDistance = leewayDistance;
    }
    public void setState(double desiredState){
        this.desiredState = desiredState;
    }
    public double updateCurrentStateAndGetOutput(double currentState){
        if(currentState > desiredState + leewayDistance){
            return -kP;
        }
        else if(currentState < desiredState - leewayDistance){
            return  kP;
        }
        else{
            return 0;
        }
    }
}
