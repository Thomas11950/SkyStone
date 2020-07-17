package org.firstinspires.ftc.teamcode.Ramsete;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class Path {
	public ArrayList<Pose> path = new ArrayList<Pose>();
	public Path(String splineName) {
		File fileContainingPoints = new File(splineName);
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
	        int powerInitialPosition = pointInStringFormat.indexOf("PowerInitial:");
	        String powerInitialString = pointInStringFormat.substring(powerInitialPosition + 13);
	        for(int i = 0; i < powerInitialString.length(); i++){
	            if(powerInitialString.substring(i,i+1).equals(" ")){
	                powerInitialString = powerInitialString.substring(0,i);
	                i=powerInitialString.length();
	            }
	        }
	        double powerInitial = Double.parseDouble(powerInitialString);
	        
	        int powerFinalPosition = pointInStringFormat.indexOf("PowerFinal:");
	        String powerFinalString = pointInStringFormat.substring(powerFinalPosition + 11);
	        for(int i = 0; i < powerFinalString.length(); i++){
	            if(powerFinalString.substring(i,i+1).equals(" ")){
	                powerFinalString = powerFinalString.substring(0,i);
	                i=powerFinalString.length();
	            }
	        }
	        double powerFinal = Double.parseDouble(powerFinalString);
	        path.add(new Pose(xCoord,yCoord,powerInitial,powerFinal));
	    }
	}
}
