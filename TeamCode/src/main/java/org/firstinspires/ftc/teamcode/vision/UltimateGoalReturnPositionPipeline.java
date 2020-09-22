package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class UltimateGoalReturnPositionPipeline extends OpenCvPipeline {
    public int BOTTOMLEFTX;
    public int BOTTOMLEFTY;//bot left coordinates of small 1 ring box
    public int WIDTH;//width of 1 ring box
    public int HEIGHT1;//height of 1 ring
    public int HEIGHT2;//height of 4 rings
    public int YThreshold; //Higher than threshold = is full yellow rect
    public int CbThreshold; //Lower than threshold = is full yellow rect
    public int stack; //0 is no ring, 1 is 1 ring, 2 is 4 ring
    public int highlightedRectBotLeftX;
    public int highlightedRectBotLeftY;
    public int highlightedRectTopRightX;
    public int highlightedRectTopRightY;
    double[] YCrCbHighlightedBox = {0,0,0};
    @Override
    public Mat processFrame(Mat input) {
        Mat yCrCbmat = new Mat();
        Imgproc.rectangle(input, new Point(highlightedRectBotLeftX,highlightedRectBotLeftY), new Point(highlightedRectTopRightX,highlightedRectTopRightY), new Scalar(255,255,255),5);
        Imgproc.cvtColor(input,yCrCbmat,Imgproc.COLOR_RGB2YCrCb);
        double[] YCrCbBox1 = getAverageYCrCb(yCrCbmat, BOTTOMLEFTX,BOTTOMLEFTY,BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1);
        double[] YCrCbBox2 = getAverageYCrCb(yCrCbmat, BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1,BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2);
        YCrCbHighlightedBox = getAverageYCrCb(yCrCbmat, highlightedRectBotLeftX,highlightedRectBotLeftY,highlightedRectTopRightX,highlightedRectTopRightY);
        if(isAboveThresholds(YCrCbBox2[0], YCrCbBox2[1], YCrCbBox2[2])){
            stack = 2;
        }
        else if(isAboveThresholds(YCrCbBox1[0], YCrCbBox1[1], YCrCbBox1[2])){
            stack = 1;
        }
        else{
            stack = 0;
        }
        return input;
    }
    public double[] getAverageYCrCb(Mat input, int botLeftCornerX, int botLeftCornerY, int topRightCornerX, int topRightCornerY){
        double yTotal=0;
        double CrTotal=0;
        double CbTotal=0;
        for(int i = botLeftCornerX; i < topRightCornerX; i++){
            for(int j = botLeftCornerY; j < topRightCornerY; j++){
                double[] yCrCb = input.get(i,j);
                yTotal = yCrCb[0];
                CrTotal = yCrCb[1];
                CbTotal = yCrCb[2];
            }
        }
        double totalArea = (topRightCornerX-botLeftCornerX)/(topRightCornerY-botLeftCornerY);
        return new double[]{yTotal/totalArea,CrTotal/totalArea,CbTotal/totalArea};
    }
    public boolean isAboveThresholds(double avgY, double avgCr, double avgCb){
        return avgY > YThreshold && avgCb < CbThreshold;
    }
}
