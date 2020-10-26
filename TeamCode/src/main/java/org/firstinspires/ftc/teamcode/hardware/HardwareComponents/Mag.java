package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Mag {
    RegServo magServo;
    RegServo ringPusher;
    double ringPusherResting = 0.54905;
    double ringPusherPushedIn = 0.424;
    double takeRingsPosition=0.4814;
    double feedTopRingPosition=0.3166;
    double feedMiddleRingPosition=0.26788;
    double feedBottomRingPosition=0.23722;
    public State currentState;
    public Mag(RegServo magServo, RegServo ringPusher){
        this.magServo = magServo;
        this.ringPusher = ringPusher;
        collectRings();
        currentState = State.COLLECT;
    }
    public void feedTopRing(){
        magServo.setPosition(feedTopRingPosition);
    }
    public void feedMidRing(){
        magServo.setPosition(feedMiddleRingPosition);
    }
    public void feedBottomRing(){
        magServo.setPosition(feedBottomRingPosition);
    }
    public void collectRings(){
        magServo.setPosition(takeRingsPosition);
    }
    public void updateStateAndSetPosition(){
        if(currentState == State.COLLECT){
            currentState = State.TOP;
        }
        else if(currentState == State.TOP){
            currentState = State.MID;
        }
        else if(currentState == State.MID){
            currentState = State.BOTTOM;
        }
        else{
            currentState = State.COLLECT;
        }
        if(currentState == State.COLLECT){
            collectRings();
        }
        else if(currentState == State.BOTTOM){
            feedBottomRing();
        }
        else if(currentState == State.MID){
            feedMidRing();
        }
        else{
            feedTopRing();
        }

    }
    public enum State{
        TOP,
        MID,
        BOTTOM,
        COLLECT
    }
    public void pushInRings(){
        ringPusher.setPosition(ringPusherPushedIn);
    }
    public void setRingPusherResting(){
        ringPusher.setPosition(ringPusherResting);
    }
}
