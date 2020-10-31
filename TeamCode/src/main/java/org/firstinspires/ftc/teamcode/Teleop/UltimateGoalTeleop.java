package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Mag;
import org.firstinspires.ftc.teamcode.vision.T265;

import java.io.FileWriter;
import java.io.IOException;
@TeleOp(name = "UltimateGoalTeleop",group="TeleOp")
public class UltimateGoalTeleop extends OpMode {
    Hardware hardware;
    boolean slowMode;
    //toggles
    boolean manuelTurretControl = true;
    boolean manuelTurretControlToggledPrevLoop = false;
    boolean magUpdateStateAndSetPositionPrevLoop = false;
    boolean manuelRampControl = true;
    boolean manuelRampControlTogglePrevLoop = false;
    boolean shooterOn = false;
    boolean shooterOnTogglePrevLoop = false;
    boolean grip = true;
    boolean gripOnToggledPrevLoop = false;
    boolean armStateToggledPrevLoop = false;
    public double shooterVelo;
    public void init(){
        /*if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(),T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);
        }*/
        hardware = new Hardware(hardwareMap,telemetry);
        slowMode = false;
        shooterVelo = -1440;
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
    public void start(){
        //T265.slamra.start();
    }
    public void loop(){
        double leftPower;
        double rightPower;
        if(gamepad1.left_trigger > 0){
            slowMode = true;
        }
        else{
            slowMode = false;
        }
        if(!slowMode) {
            double leftAbsValue = Math.abs(gamepad1.left_stick_y);
            double rightAbsValue = Math.abs(gamepad1.right_stick_y);
            leftPower = logistic(leftAbsValue, 1, 7.2) * -gamepad1.left_stick_y / leftAbsValue;
            rightPower = logistic(rightAbsValue, 1, 7.2) * -gamepad1.right_stick_y / rightAbsValue;
        }
        else{
            leftPower = -gamepad1.left_stick_y*0.5;
            rightPower = -gamepad1.right_stick_y*0.5;
        }
        if(gamepad1.b){
            leftPower=0.3;
            rightPower=-0.3;
        }
        hardware.sixWheelDrive.LF.setPower(leftPower);
        hardware.sixWheelDrive.LB.setPower(leftPower);
        hardware.sixWheelDrive.RF.setPower(leftPower);
        hardware.sixWheelDrive.RB.setPower(leftPower);
        hardware.sendT265OdoData= false;
        hardware.loop();
        /*T265Camera.CameraUpdate up = T265.slamra.getLastReceivedCameraUpdate();
        double[] t265position = T265.getCameraPosition(up);
        if(up.confidence == null){
            telemetry.addLine("no confidence level yet");
        }
        if(up.confidence == T265Camera.PoseConfidence.Failed){
            telemetry.addLine("Pose Confidence Failed");
        }
        else if(up.confidence == T265Camera.PoseConfidence.Medium){
            telemetry.addLine("Pose Confidence Medium");
        }
        else if(up.confidence == T265Camera.PoseConfidence.High){
            telemetry.addLine("Pose Confidence High");
        }
        else{
            telemetry.addLine("Pose Confidence Low");
        }*/
        //manuel turret control toggle & turret control
        if(gamepad2.a) {
            if(!manuelTurretControlToggledPrevLoop) {
                manuelTurretControl = !manuelTurretControl;
            }
            manuelTurretControlToggledPrevLoop = true;
        }
        else{
            if(manuelTurretControlToggledPrevLoop){
                manuelTurretControlToggledPrevLoop = false;
            }
        }
        if(manuelTurretControl){
            hardware.turret.updatePID = false;
            telemetry.addLine("manuel turret control on rn");
            hardware.turret.setAllTurretServoPowers(gamepad2.left_stick_x);
        }
        else{
            hardware.turret.updatePID = true;
            hardware.turret.pointTowardsHighGoal();
        }
        telemetry.addData("turret Position",hardware.turret.encoder.getCurrentPosition());
        //intake control
        hardware.intake.turnIntake(gamepad1.right_trigger);
        //mag control
        if(gamepad1.right_bumper) {
            if(!magUpdateStateAndSetPositionPrevLoop) {
                hardware.mag.updateStateAndSetPosition();
            }
            magUpdateStateAndSetPositionPrevLoop = true;
        }
        else{
            if(magUpdateStateAndSetPositionPrevLoop){
                magUpdateStateAndSetPositionPrevLoop = false;
            }
        }
        //ramp manuel control and automatic control
        if(gamepad2.b) {
            if(!manuelRampControlTogglePrevLoop) {
                manuelRampControl = !manuelRampControl;
            }
            manuelRampControlTogglePrevLoop = true;
        }
        else{
            if(manuelRampControlTogglePrevLoop){
                manuelRampControlTogglePrevLoop = false;
            }
        }
        if(manuelRampControl){
            hardware.shooter.setRampPosition(hardware.shooter.rampPostion - gamepad2.right_stick_y*0.001);
        }
        else{
            hardware.shooter.autoRampPositionForHighGoal(Math.hypot(hardware.getYAbsoluteCenter()- FieldConstants.highGoalPosition[1],hardware.getXAbsoluteCenter() - FieldConstants.highGoalPosition[0]));
        }

        //shooter
        if(gamepad1.left_bumper) {
            if(!shooterOnTogglePrevLoop) {
                shooterOn = !shooterOn;
                if(shooterOn){
                    hardware.shooter.firstUpdateShooterPIDFLoop = true;
                }
            }
            shooterOnTogglePrevLoop = true;
        }
        else{
            if(shooterOnTogglePrevLoop){
                shooterOnTogglePrevLoop = false;
            }
        }
        if(shooterOn){
            hardware.shooter.updatePID = true;
            hardware.shooter.shooterVeloPID.setState(shooterVelo);
        }
        else{
            hardware.shooter.updatePID = false;
            hardware.shooter.shooterMotor2.setPower(0);
            hardware.shooter.shooterMotor1.setPower(0);
        }
        //Flicker
        if(gamepad2.x){
            hardware.mag.pushInRings();
        }
        if(gamepad2.y){
            hardware.mag.setRingPusherResting();
        }
        //wobbler
        if(gamepad2.left_bumper) {
            if(!gripOnToggledPrevLoop) {
                grip = !grip;
            }
            gripOnToggledPrevLoop = true;
        }
        else{
            if(gripOnToggledPrevLoop){
                gripOnToggledPrevLoop = false;
            }
        }
        if(grip){
            hardware.wobbler.gripWobble();
        }else{
            hardware.wobbler.releaseWobble();
        }
        if(gamepad2.right_bumper) {
            if(!armStateToggledPrevLoop) {
                hardware.wobbler.toggleArmState();
            }
            armStateToggledPrevLoop = true;
        }
        else{
            if(armStateToggledPrevLoop){
                armStateToggledPrevLoop = false;
            }
        }
        if(gamepad1.dpad_up){
            shooterVelo -= 0.5;
        }else if(gamepad1.dpad_down){
            shooterVelo += 0.5;
        }
        telemetry.addData("shooter On",shooterOn);
        if(hardware.mag.currentState == Mag.State.BOTTOM){
            telemetry.addLine("Mag State: BOTTOM");
        }
        else if(hardware.mag.currentState == Mag.State.MID){
            telemetry.addLine("Mag State: MID");
        }
        else if(hardware.mag.currentState == Mag.State.BOTTOM){
            telemetry.addLine("Mag State: BOTTOM");
        }
        else{
            telemetry.addLine("Mag State: COLLECT");
        }
        telemetry.addData("Wobbler grip",grip);
        telemetry.addData("Flap position",hardware.shooter.rampPostion);
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+Math.toDegrees(hardware.angle1) + ", angle 2: "+Math.toDegrees(hardware.angle2));
        telemetry.addLine("XCenter: " + hardware.getXAbsoluteCenter()  + ", YCenter: "+hardware.getYAbsoluteCenter());
        telemetry.addLine("left position: " + hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + -hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("shooter velo: "+shooterVelo);
        telemetry.addLine("turret Angle: "+Math.toDegrees(hardware.turret.localTurretAngleRadians())+", turret output power: "+gamepad2.left_stick_x);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
        telemetry.update();
    }
    public void stop(){
        //T265.slamra.stop();
    }
}
