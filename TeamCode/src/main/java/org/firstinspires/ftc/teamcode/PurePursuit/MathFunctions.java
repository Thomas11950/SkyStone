package org.firstinspires.ftc.teamcode.PurePursuit;

import java.util.ArrayList;

public class MathFunctions {
    public static double[] findIntercept(double equation1Slope, double equation1YIntercept, double equation2Slope, double equation2YIntercept){
        double x = (equation2YIntercept-equation1YIntercept)/(equation1Slope-equation2Slope);
        double y = equation1Slope * x + equation1YIntercept;
        double[] point = new double[2];
        point[0] = x;
        point[1] = y;
        return point;
    }
    public static boolean isInBetween(double startNum, double endNum, double testPoint){
        if(startNum > endNum){
            double startNumStorage = startNum;
            startNum = endNum;
            endNum = startNumStorage;
        }
        return testPoint <= endNum && testPoint >= startNum;
    }
    public static double[] lineToPointDistanceAndClosestPoint(double startPointX, double startPointY, double endPointX, double endPointY, double setPointX, double setPointY){
        double slope = (endPointY-startPointY)/(endPointX-startPointX);
        double yIntercept = startPointY - slope * startPointX;
        double inverseSlope = -1/slope;
        double inverseYIntercept = setPointY - inverseSlope * setPointX;
        double[] intercept = findIntercept(slope, yIntercept, inverseSlope, inverseYIntercept);
        if(isInBetween(startPointX, endPointX, intercept[0])){
            double[] outputData = new double[3];
            outputData[0] = Math.hypot((intercept[0] - setPointX), (intercept[1] - setPointY));
            outputData[1] = intercept[0];
            outputData[2] = intercept[1];
            return outputData;
        }
        else{
            double startPointDist = Math.hypot((startPointX-setPointX), (startPointY-setPointY));
            double endPointDist = Math.hypot((endPointX-setPointX), (endPointY-setPointY));
            if(startPointDist < endPointDist){
                double[] outputData = new double[3];
                outputData[0] = startPointDist;
                outputData[1] = startPointX;
                outputData[2] = endPointX;
                return outputData;
            }
            else{
                double[] outputData = new double[3];
                outputData[0] = endPointDist;
                outputData[1] = endPointX;
                outputData[2] = endPointY;
                return  outputData;
            }
        }
    }
    public static double findPercentageOfCurrentLineTravelled(double startPointX, double startPointY, double endPointX, double endPointY, double setPointX, double setPointY){
        return Math.hypot(setPointX - startPointX, setPointY-startPointY)/Math.hypot(endPointX-startPointX,endPointY-startPointY);
    }
}