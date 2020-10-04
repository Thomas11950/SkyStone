package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class YCrCbValueGetterPipeline extends OpenCvPipeline {
    double BLx;
    double BLy;
    double TRx;
    double TRy;
    public double[] yCrCbHighlightedBox = new double[]{0,0,0};
    public YCrCbValueGetterPipeline(double BLx, double BLy, double TRx, double TRy){
        super();
        this.BLx=BLx;
        this.BLy=BLy;
        this.TRx=TRx;
        this.TRy=TRy;
    }
    public Mat processFrame(Mat input) {
        Mat yCrCbmat = new Mat();
        Imgproc.cvtColor(input,yCrCbmat,Imgproc.COLOR_RGB2YCrCb);
        double[] YCrCbBox1 = Core.mean(yCrCbmat.submat(new Rect(new Point(BLx,BLy), new Point(TRx,TRy)))).val;
        if(YCrCbBox1==null){
            yCrCbHighlightedBox=new double[]{0,0,0};
        }
        else {
            yCrCbHighlightedBox = YCrCbBox1;
        }
        Imgproc.rectangle(input,new Point(BLx,BLy), new Point(TRx,TRy),new Scalar(255,0,0));
        return input;
    }
    public double[] getAverageYCrCb(Mat input, int botLeftCornerX, int botLeftCornerY, int topRightCornerX, int topRightCornerY){
        double yTotal=0;
        double CrTotal=0;
        double CbTotal=0;
        for(int i = botLeftCornerX; i < topRightCornerX; i++){
            for(int j = botLeftCornerY; j < topRightCornerY; j++){
                double[] yCrCb = input.get(i,j);
                if(yCrCb == null){
                }
                else{
                    yTotal += yCrCb[0];
                    CrTotal += yCrCb[1];
                    CbTotal += yCrCb[2];
                }
            }
        }
        double totalArea = (topRightCornerX-botLeftCornerX)*(topRightCornerY-botLeftCornerY);
        return new double[]{yTotal/totalArea,CrTotal/totalArea,CbTotal/totalArea};
    }
}
