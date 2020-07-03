package org.firstinspires.ftc.teamcode.Ramsete;

import java.util.ArrayList;

public class TimeBasedTaskList {
	ArrayList<Task> taskList;
	public TimeBasedTaskList() {
		taskList = new ArrayList<Task>();
	}
	public void addTask(Task task) {
		taskList.add(task);
	}
	public double getTotalTime(){
		double totalTime = 0;
		for(Task t: taskList){
			totalTime += t.timeTaken;
		}
		return totalTime;
	}
	public MotionData getMotionData(double timeStamp) {
		double runningTime = 0;
		for(int i = 0; i < taskList.size(); i++) {
			runningTime+=taskList.get(i).timeTaken;
			if(runningTime > timeStamp) {
				runningTime-=taskList.get(i).timeTaken;
				return new MotionData(taskList.get(i).getDesiredHeading(timeStamp-runningTime), taskList.get(i).getDesiredAngularVelocity(timeStamp-runningTime), taskList.get(i).getDesiredPosition(timeStamp - runningTime), taskList.get(i).getDesiredVelocity(timeStamp-runningTime));
			}
		}
		return new MotionData(0,0,new Point(0,0),0);
	}
}
