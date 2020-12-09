package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SixWheelDriveBenjaminLinearPractice extends LinearOpMode {
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    public CRServo TurretL = null;
    public CRServo TurretR = null;

    double power = 1.0;

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode(){
        RF = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        LF = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        LB = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        RB = hardwareMap.get(DcMotor.class, "RightBackDrive");

        TurretL = hardwareMap.get(CRServo.class, "TurretLeft");
        TurretR = hardwareMap.get(CRServo.class,"TurretRight");
        while(!isStopRequested())
        {
            if(gamepad1.right_trigger > 0)
                power = 0.3;

            RF.setPower(gamepad1.right_stick_y * power);
            RB.setPower(gamepad1.right_stick_y * power);
            LF.setPower(gamepad1.left_stick_y * power);
            LB.setPower(gamepad1.left_stick_y * power);

            TurretL.setPower(gamepad2.left_stick_x);
            TurretR.setPower(gamepad2.left_stick_x);

            power = 1;

            telemetry.addLine("right motor position: " + RF.getCurrentPosition());
            telemetry.addLine("left motor position: " + LF.getCurrentPosition());
            telemetry.update();
        }
    }
}
