package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SixWheelDriveBenjaminFullPractice extends OpMode {
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;

    public CRServo TurretL = null;
    public CRServo TurretR = null;

    double power = 1.0;

    @Override
    public void init() {
        RF = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        LF = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        LB = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        RB = hardwareMap.get(DcMotor.class, "RightBackDrive");

        TurretL = hardwareMap.get(CRServo.class, "TurretLeft");
        TurretR = hardwareMap.get(CRServo.class,"TurretRight");

        RF.setPower(0);
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        TurretL.setPower(0);
        TurretR.setPower(0);

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
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
