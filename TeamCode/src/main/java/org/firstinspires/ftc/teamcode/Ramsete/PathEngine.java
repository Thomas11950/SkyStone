package org.firstinspires.ftc.teamcode.Ramsete;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.sql.Array;
import java.sql.Time;
import java.util.ArrayList;



public class PathEngine{
	Path pathToDraw;
	String pathFileName;
	public double maxAccel;
	public double maxAngularAccel= Math.toRadians(0.005);
	public double[] prevIntercept;
	ArrayList<ArrayList<Point>> arcs;
	TimeBasedTaskList taskList;
	org.firstinspires.ftc.teamcode.hardware.Hardware hardware;
	public PathEngine(double maxAccel, double maxAngularAccel, String pathFileName, Hardware hardware){
		this.maxAccel = maxAccel;
		this.maxAngularAccel =maxAngularAccel;
		this.pathFileName = pathFileName;
		taskList = new TimeBasedTaskList();
		this.hardware = hardware;
	}
	public void init() {
		pathToDraw = new Path(pathFileName);
		arcs = new ArrayList<ArrayList<Point>>();
		Double[] angles = new Double[pathToDraw.path.size()-1];
		for(int i = 0; i< angles.length; i++) {
			angles[i] = Math.atan2(pathToDraw.path.get(i+1).Y - pathToDraw.path.get(i).Y, pathToDraw.path.get(i+1).X - pathToDraw.path.get(i).X);
		}
		prevIntercept[0] = pathToDraw.path.get(0).X;
		prevIntercept[1] = pathToDraw.path.get(0).Y;
		for(int i = 0; i < pathToDraw.path.size()-2;i++) {
			if(Math.abs(MathFunctions.angleFormatting(angles[i]-angles[i+1])) > Math.toRadians(0.1)) {
				System.out.println("we're now on the " + i +"th arc!");
				simulateArc arc = new simulateArc(pathToDraw.path.get(i).X, pathToDraw.path.get(i).Y, pathToDraw.path.get(i+1).powerInitial, pathToDraw.path.get(i+1).powerFinal, MathFunctions.angleFormatting(angles[i+1]-angles[i]),angles[i],maxAngularAccel);
				arc.calculateArc();
				double[] intercept = new double[2];
				double arcEndPointX = arc.arc.get(arc.arc.size()-1).X;
				double arcEndPointY = arc.arc.get(arc.arc.size()-1).Y;
				if(pathToDraw.path.get(i+1).X != pathToDraw.path.get(i).X && pathToDraw.path.get(i+1).X != pathToDraw.path.get(i+2).X) {
					double slopeLine1 = 1.0*(pathToDraw.path.get(i+1).Y-pathToDraw.path.get(i).Y)/(pathToDraw.path.get(i+1).X - pathToDraw.path.get(i).X);
					System.out.println("slopeLine1: "+slopeLine1);
					double interceptLine1 = arcEndPointY - slopeLine1 * arcEndPointX;
					double slopeLine2 = 1.0*(pathToDraw.path.get(i+2).Y-pathToDraw.path.get(i+1).Y)/(pathToDraw.path.get(i+2).X - pathToDraw.path.get(i+1).X);
					System.out.println("slopeLine2: "+slopeLine2);
					double interceptLine2 = pathToDraw.path.get(i+1).Y - slopeLine2 * pathToDraw.path.get(i+1).X;
					intercept = MathFunctions.findIntercept(slopeLine1, interceptLine1, slopeLine2, interceptLine2);
				}
				else if(pathToDraw.path.get(i+1).X == pathToDraw.path.get(i).X){
					intercept = new double[2];
					intercept[0] = arcEndPointX;
					System.out.println("i+2 X: " + pathToDraw.path.get(i+2).X + ", i+1 X: " + pathToDraw.path.get(i+1).X);
					double slopeLine2 = (pathToDraw.path.get(i+2).Y-pathToDraw.path.get(i+1).Y)/(pathToDraw.path.get(i+2).X - pathToDraw.path.get(i+1).X);
					double interceptLine2 = pathToDraw.path.get(i+1).Y - slopeLine2 * pathToDraw.path.get(i+1).X;
					intercept[1] = slopeLine2 * intercept[0] + interceptLine2;
				}
				else if(pathToDraw.path.get(i+1).X == pathToDraw.path.get(i+2).X){
					intercept = new double[2];
					double slopeLine1 = (pathToDraw.path.get(i+1).Y-pathToDraw.path.get(i).Y)/(pathToDraw.path.get(i+1).X - pathToDraw.path.get(i).X);
					double interceptLine1 = arcEndPointY - slopeLine1 * arcEndPointX;
					intercept[0] = pathToDraw.path.get(i+1).X;
					intercept[1] = slopeLine1 * intercept[0] + interceptLine1;
				}
				System.out.println("intercept X: " + intercept[0] + ", intercept Y: " + intercept[1]);
				double translateX = intercept[0] - arcEndPointX;
				double translateY = intercept[1] - arcEndPointY;
				for(Point p: arc.arc) {
					p.X += translateX;
					p.Y += translateY;
				}
				for(Point p: arc.arcHalf1) {
					p.X += translateX;
					p.Y += translateY;
				}
				for(Point p: arc.arcHalf2) {
					p.X += translateX;
					p.Y += translateY;
				}
				double lineEndX = arc.arc.get(0).X;
				double lineEndY = arc.arc.get(0).Y;
				double lineLength = Math.hypot(lineEndX - prevIntercept[0],lineEndY-prevIntercept[1]);
				double lineAccel = (Math.pow(pathToDraw.path.get(i+1).powerInitial,2) - Math.pow(pathToDraw.path.get(i).powerFinal,2))/(2*lineLength);
				taskList.addTask(new Task(Math.abs(2*lineLength/(pathToDraw.path.get(i).powerFinal + pathToDraw.path.get(i+1).powerInitial)), pathToDraw.path.get(i).powerFinal,lineAccel,new Point(prevIntercept[0],prevIntercept[1]),angles[i]));
				double arcTime = arc.turnTime(arc.deltaHeading,arc.maxAngularAccel);
				double arcAccel = (pathToDraw.path.get(i+1).powerFinal - pathToDraw.path.get(i+1).powerInitial)/(2*arcTime);
				taskList.addTask(new ArcTask(arcTime, pathToDraw.path.get(i+1).powerInitial, arcAccel,arc.arcHalf1.get(0),  angles[i],0,arc.maxAngularAccel,arc.arcHalf1));
				taskList.addTask(new ArcTask(arcTime, pathToDraw.path.get(i+1).powerInitial + arcAccel * arcTime,arcAccel,arc.arcHalf2.get(0),arc.halfTimeAngle,arc.halfTimeAngularVelocity,-arc.maxAngularAccel,arc.arcHalf2 ));
				prevIntercept = intercept;
				arcs.add(i, arc.arc);
			}
			else {
				double lineEndX = pathToDraw.path.get(i+1).X;
				double lineEndY = pathToDraw.path.get(i+1).Y;
				double lineLength = Math.hypot(lineEndX - prevIntercept[0],lineEndY-prevIntercept[1]);
				double lineAccel = (Math.pow(pathToDraw.path.get(i+1).powerInitial,2) - Math.pow(pathToDraw.path.get(i).powerFinal,2))/(2*lineLength);
				taskList.addTask(new Task(Math.abs(2*lineLength/(pathToDraw.path.get(i).powerFinal + pathToDraw.path.get(i+1).powerInitial)), pathToDraw.path.get(i).powerFinal,lineAccel,new Point(prevIntercept[0],prevIntercept[1]),angles[i]));
				prevIntercept[0] = lineEndX;
				prevIntercept[1] = lineEndY;
				arcs.add(null);
			}
		}
		double lineEndX = pathToDraw.path.get(pathToDraw.path.size()-1).X;
		double lineEndY = pathToDraw.path.get(pathToDraw.path.size()-1).Y;
		double lineLength = Math.hypot(lineEndX - prevIntercept[0],lineEndY-prevIntercept[1]);
		double lineAccel = (Math.pow(pathToDraw.path.get(pathToDraw.path.size()-1).powerInitial,2) - Math.pow(pathToDraw.path.get(pathToDraw.path.size()-2).powerFinal,2))/(2*lineLength);
		taskList.addTask(new Task(Math.abs(2*lineLength/(pathToDraw.path.get(pathToDraw.path.size()-2).powerFinal + pathToDraw.path.get(pathToDraw.path.size()-1).powerInitial)), pathToDraw.path.get(pathToDraw.path.size()-2).powerFinal,lineAccel,new Point(prevIntercept[0],prevIntercept[1]),angles[angles.length-1]));
		prevIntercept[0] = lineEndX;
		prevIntercept[1] = lineEndY;
		arcs.add(null);
	}
	public void run(ElapsedTime time, double constantB, double constantC){
		double startTime = time.milliseconds();
		while((time.milliseconds() - startTime)/1000 < taskList.getTotalTime()){
			double currentTime = (time.milliseconds() - startTime)/1000;
			MotionData currentMotionData = taskList.getMotionData(currentTime);
			double xErrorLocal = (currentMotionData.desiredPosition.X - hardware.getX()) * Math.cos(hardware.angle) + (currentMotionData.desiredPosition.Y - hardware.getY()) * Math.sin(hardware.angle);
			double yErrorLocal = (currentMotionData.desiredPosition.X - hardware.getX()) * -Math.sin(hardware.angle) + (currentMotionData.desiredPosition.Y - hardware.getY()) * Math.cos(hardware.angle);
			double headingError = currentMotionData.desiredHeading - hardware.angle;
			double constantK = MathFunctions.getConstantK(currentMotionData.desiredVelocity, currentMotionData.desiredAngularVelocity,  constantB,  constantC);
			double angularVelocityCommand = MathFunctions.getAngularVelocityCommand(currentMotionData.desiredVelocity, currentMotionData.desiredAngularVelocity, headingError, yErrorLocal, constantB, constantK);
			double velocityCommand = MathFunctions.velocityCommand(currentMotionData.desiredVelocity, headingError, xErrorLocal, constantK);
			hardware.sixWheelDrive.setPowers(velocityCommand,angularVelocityCommand);
		}
	}
}
