package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Mag {
    RegServo magServo;
    double takeRingsPosition=0.47444;
    double feedTopRingPosition=0.29989;
    double feedMiddleRingPosition=0.26624;
    double feedBottomRingPosition=0.2317;
    public State currentState;
    public Mag(RegServo magServo){
        this.magServo = magServo;
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
            currentState = State.BOTTOM;
        }
        else if(currentState == State.BOTTOM){
            currentState = State.MID;
        }
        else if(currentState == State.MID){
            currentState = State.TOP;
        }
        else{
            currentState = State.COLLECT;
        }
        if(currentState == State.COLLECT){
            collectRings();
        }
        else if(currentState == State.BOTTOM){
            feedMidRing();
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
}
