package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Ramsete.PathEngine;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

@TeleOp(name="TurningPIDTester", group="TeleOp")
public class TurningPIDtester extends LinearOpMode {
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        waitForStart();
        hardware.sendT265OdoData = false;
        TurretPID headingPID = new TurretPID(1.2,6,0.12,Math.toRadians(20), hardware.time);
        headingPID.setState(Math.toRadians(90));
        while(!isStopRequested()) {
            telemetry.addData("heading: ", Math.toDegrees(hardware.angle));
            double output = headingPID.updateCurrentStateAndGetOutput(hardware.angle);
            telemetry.addData("output: ",output);
            telemetry.addData("currentIntegral: ",headingPID.integral);
            telemetry.update();
            hardware.sixWheelDrive.turn(output);
            hardware.loop();
        }
    }
}
