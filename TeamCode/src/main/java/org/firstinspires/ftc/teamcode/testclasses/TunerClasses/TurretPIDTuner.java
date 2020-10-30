package org.firstinspires.ftc.teamcode.testclasses.TunerClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Turret;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;
@Autonomous(name = "TurretPIDTuner", group="Autonomous")
public class TurretPIDTuner extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        waitForStart();
        hardware.sendT265OdoData = false;
        TurretPID turretPID = new TurretPID(1.2,6,0.12,Math.toRadians(20), hardware.time);
        turretPID.setState(Math.toRadians(90));
        while(!isStopRequested()) {
            telemetry.addData("heading: ", Math.toDegrees(hardware.turret.localTurretAngleRadians()));
            double output = turretPID.updateCurrentStateAndGetOutput(hardware.turret.localTurretAngleRadians());
            telemetry.addData("output: ",output);
            telemetry.addData("currentIntegral: ",turretPID.integral);
            telemetry.update();
            hardware.turret.setAllTurretServoPowers(output);
            hardware.loop();
        }
    }
}
