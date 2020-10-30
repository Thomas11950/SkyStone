package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class UltimateGoalReturnPositionPipeline extends OpenCvPipeline {
    public int BOTTOMLEFTX = 533;
    public int BOTTOMLEFTY = 233;//bot left coordinates of small 1 ring box
    public int WIDTH = 79;//width of 1 ring box
    public int HEIGHT1 = 33;//height of 1 ring
    public int BUFFER = 12;
    public int HEIGHT2 = 51;//height of 4 rings
    public double HThresholdLow = 7;
    public double HThresholdHigh = 13;
    public int stack;
    @Override
    public Mat processFrame(Mat input) {
        Mat HSVmat = new Mat();
        Imgproc.rectangle(input, new Point(BOTTOMLEFTX,BOTTOMLEFTY), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1), new Scalar(255,255,255),1);
        Imgproc.rectangle(input, new Point(BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1+BUFFER), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2), new Scalar(255,255,255),1);
        Imgproc.cvtColor(input,HSVmat,Imgproc.COLOR_RGB2HSV);
        double[] topBox = Core.mean(HSVmat.submat(new Rect(new Point(BOTTOMLEFTX,BOTTOMLEFTY), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1)))).val;
                //getAverageYCrCb(yCrCbmat, BOTTOMLEFTX,BOTTOMLEFTY,BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1);
        double[] bottomBox = Core.mean(HSVmat.submat(new Rect(new Point(BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1+BUFFER), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2)))).val;
                //getAverageYCrCb(yCrCbmat, BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1,BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2);
        Imgproc.putText(input,"TopBoxH: "+topBox[0],new Point(0,100),1,1,new Scalar(0,0,0));
        Imgproc.putText(input,"BottomBoxH: "+bottomBox[0],new Point(0,200),1,1,new Scalar(0,0,0));
        if(isAboveThresholds(topBox[0])){
            stack = 2;
        }
        else if(isAboveThresholds(bottomBox[0])){
            stack = 1;
        }
        else{
            stack = 0;
        }
        return input;
    }

    public boolean isAboveThresholds(double avgH){
        return avgH > HThresholdLow && avgH < HThresholdHigh;
    }
}
