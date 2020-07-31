package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareThreadInterface;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class SplineRunner {
    String fileNameForPoints;
    File fileContainingPoints;
    ArrayList<Pose> spline = new ArrayList<Pose>();
    Hardware hardware;
    LinearOpMode parentOP;
    public SplineRunner(String fileNameForPoints, Hardware hardware, LinearOpMode parentOP){
        this.parentOP = parentOP;
        this.hardware = hardware;
        this.fileNameForPoints = fileNameForPoints;
        fileContainingPoints = new File(this.fileNameForPoints);
        Scanner scnr;
        try {
            scnr = new Scanner(fileContainingPoints);
        }
        catch(FileNotFoundException e){
            return;
        }
        while(scnr.hasNextLine()){
            String pointInStringFormat = scnr.nextLine();
            int xCoordPosition = pointInStringFormat.indexOf("X:");
            String xCoordString = pointInStringFormat.substring(xCoordPosition + 2);
            for(int i = 0; i < xCoordString.length(); i++){
                if(xCoordString.substring(i,i+1).equals(" ")){
                    xCoordString = xCoordString.substring(0,i);
                    i=xCoordString.length();
                }
            }
            double xCoord = Double.parseDouble(xCoordString);
            int yCoordPosition = pointInStringFormat.indexOf("Y:");
            String yCoordString = pointInStringFormat.substring(yCoordPosition + 2);
            for(int i = 0; i < yCoordString.length(); i++){
                if(yCoordString.substring(i,i+1).equals(" ")){
                    yCoordString = yCoordString.substring(0,i);
                    i=yCoordString.length();
                }
            }
            double yCoord = Double.parseDouble(yCoordString);
            int headingposition = pointInStringFormat.indexOf("Heading:");
            String headingString = pointInStringFormat.substring(headingposition + 8);
            for(int i = 0; i < headingString.length(); i++){
                if(headingString.substring(i,i+1).equals(" ")){
                    headingString = headingString.substring(0,i);
                    i=headingString.length();
                }
            }
            double heading = Integer.parseInt(headingString);
            int translationPowerPosition = pointInStringFormat.indexOf("TranslationPower:");
            String translationPowerString = pointInStringFormat.substring(translationPowerPosition + 17);
            for(int i = 0; i < translationPowerString.length(); i++){
                if(translationPowerString.substring(i,i+1).equals(" ")){
                    translationPowerString = translationPowerString.substring(0,i);
                    i=translationPowerString.length();
                }
            }
            double translationPower = Double.parseDouble(headingString);
            int turningPowerPosition = pointInStringFormat.indexOf("TurnPower:");
            String turnPowerString = pointInStringFormat.substring(turningPowerPosition + 10);
            for(int i = 0; i < turnPowerString.length(); i++){
                if(turnPowerString.substring(i,i+1).equals(" ")){
                    turnPowerString = turnPowerString.substring(0,i);
                    i=turnPowerString.length();
                }
            }
            double turnPower = Double.parseDouble(headingString);
            spline.add(new Pose(xCoord,yCoord,heading,translationPower,turnPower));
        }
    }
    public int getSplinePoint(){
        double minDist = Math.hypot(spline.get(0).X, spline.get(0).Y);
        int minDistIndex = 0;
        int i;
        for(i = 1; i < spline.size();i++){
            double currentDist = Math.hypot(spline.get(i).X,spline.get(i).Y);
            if(currentDist < minDist){
                minDist = currentDist;
                minDistIndex = i;
            }
        }
        return minDistIndex;
    }
    public int getTargetPoint(int STEP_SIZE){
        int p = getSplinePoint();
        if(p < spline.size() - STEP_SIZE){
            return p + STEP_SIZE;
        }
        else{
            return spline.size()-1;
        }
    }
    public void start(double endMarginOfError, double lookAheadDist, Gamepad g){
        boolean splineFinished = false;
        Pose absoluteEndPoint = spline.get(spline.size() - 1);
        while(!splineFinished && !parentOP.isStopRequested() && !g.a){
            double[][] allData = new double[spline.size()-1][3];
            for(int i = 0; i < spline.size() - 1;i++){
                double[] lineToCurrentPointData = MathFunctions.lineToPointDistanceAndClosestPoint(spline.get(i).X, spline.get(i).Y,spline.get(i+1).X,spline.get(i+1).Y,hardware.getX(),hardware.getY());
                allData[i] = lineToCurrentPointData;
            }
            double minDist = allData[0][0];
            int minDistLine = 0;
            for(int p = 1; p < allData.length; p++){
                if(allData[p][0] < minDist){
                    minDist = allData[p][0];
                    minDistLine = p;
                }
            }
            double percentOfCurrentLineTravelled = MathFunctions.findPercentageOfCurrentLineTravelled(spline.get(minDistLine).X,spline.get(minDistLine).Y, spline.get(minDistLine+1).X,spline.get(minDistLine+1).Y,allData[minDistLine][1],allData[minDistLine][2]);
            double[] targetPoint = getTargetPoint(minDistLine, percentOfCurrentLineTravelled, lookAheadDist);
            double targetX = spline.get((int)targetPoint[0]).X + (spline.get((int)targetPoint[0] + 1).X - spline.get((int)targetPoint[0]).X) * targetPoint[1];
            double targetY = spline.get((int)targetPoint[0]).Y + (spline.get((int)targetPoint[0] + 1).Y - spline.get((int)targetPoint[0]).Y) * targetPoint[1];
            setPowersForTargetPoint6wd(spline.get(minDistLine), targetX,targetY);
            if(Math.hypot(spline.get(spline.size()-1).X - hardware.getX(), spline.get(spline.size()-1).Y - hardware.getY()) < endMarginOfError){
                splineFinished = true;
            }
        }
    }
    public double[] getTargetPoint(int currentLine, double percentOfCurrentLineFinished, double lookAheadDist){
        double runningTotal = 0;
        double remainingAmountOfCurrentLine = (1-percentOfCurrentLineFinished) * getLineLength(currentLine);
        runningTotal += remainingAmountOfCurrentLine;
        if(runningTotal > lookAheadDist){
            double lookAheadPercentOfCurrentLine = lookAheadDist / getLineLength(currentLine);
            double[] toReturn = new double[2];
            toReturn[0] = currentLine;
            toReturn[1] = percentOfCurrentLineFinished + lookAheadPercentOfCurrentLine;
            return toReturn;
        }
        else{
            for(int i = currentLine + 1; i < spline.size() -1; i++){
                double runningTotalPrev = runningTotal;
                runningTotal += getLineLength(i);
                if(runningTotal > lookAheadDist){
                    double lookAheadPercentOfCurrentLine = (lookAheadDist-runningTotalPrev) / getLineLength(i);
                    double[] toReturn = new double[2];
                    toReturn[0] = i;
                    toReturn[1] = lookAheadPercentOfCurrentLine;
                    return toReturn;
                }
            }
            double[] toReturn = new double[2];
            toReturn[0] = spline.size()-2;
            toReturn[1] = 1;
            return toReturn;
        }
    }
    public double getLineLength(int linePosition){
        return Math.hypot(spline.get(linePosition).X - spline.get(linePosition + 1).X, spline.get(linePosition).Y - spline.get(linePosition + 1).Y);
    }
    /*public void setPowersForTargetPoint(Pose currentPoint, double targetX, double targetY){
        double targetAngle = Math.atan2(targetY-hardware.getY(),targetX-hardware.getX());
        double currentHeading = hardware.angle;
        double deltaHeading = MathFunctions.keepAngleWithin180Degrees(targetAngle-currentHeading);
        double movementY = currentPoint.translationPower*Math.cos(deltaHeading);
        double movementX = currentPoint.translationPower*Math.sin(deltaHeading);
        double deltaHeadingToTurn = MathFunctions.keepAngleWithin180Degrees(currentPoint.heading- currentHeading);
        double turn = Range.clip(deltaHeadingToTurn,-1,1);
        hardware.sixWheelDrive.setPowersNonTank(movementY, turn*currentPoint.rotationPower);
    }*/
    public void setPowersForTargetPoint6wd(Pose currentPoint, double targetX, double targetY){
        double targetAngle = Math.atan2(targetY-hardware.getY(),targetX-hardware.getX());
        double currentHeading = hardware.angle;
        double deltaHeading = MathFunctions.keepAngleWithin180Degrees(targetAngle-currentHeading);
        double movementY = 0.6*Math.cos(deltaHeading);
        RobotLog.dd("PPDEBUG","Movement Y: "+movementY + ", TranslationPower: " + currentPoint.translationPower);
        double turn = Range.clip(deltaHeading,-1,1);
        hardware.sixWheelDrive.setPowersNonTank(movementY, turn*0.7);
    }
}