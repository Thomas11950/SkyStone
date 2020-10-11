package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvCameraRotation;
@TeleOp(name="VisionValueGetter", group="TeleOp")
public class VisionValueGetter extends LinearOpMode {
    OpenCvCamera webcam;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        waitForStart();
        VisionValueGetterPipeline pipeline = new VisionValueGetterPipeline(310,230,330,250);
        webcam.setPipeline(pipeline);
        webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webcam.resumeViewport();
        while(!isStopRequested()){
            telemetry.addLine("1: "+ pipeline.highlightedBox[0] + ", 2: "+pipeline.highlightedBox[1] + ", 3: "+pipeline.highlightedBox[2]);
            telemetry.addLine("range for 1: " + pipeline.rangeValue1[0] + " to " + pipeline.rangeValue1[1]);
            telemetry.addLine("range for 2: " + pipeline.rangeValue2[0] + " to " + pipeline.rangeValue2[1]);
            telemetry.addLine("range for 3: " + pipeline.rangeValue3[0] + " to " + pipeline.rangeValue3[1]);
            telemetry.update();
        }
    }
}
