package org.firstinspires.ftc.teamcode.Ramsete;

public class MotionData {
	public double desiredHeading;
	public double desiredAngularVelocity;
	public double desiredVelocity;
	public Point desiredPosition;
	public MotionData(double desiredHeading, double desiredAngularVelocity, Point desiredPosition, double desiredVelocity) {
		this.desiredHeading = desiredHeading;
		this.desiredAngularVelocity = desiredAngularVelocity;
		this.desiredPosition = desiredPosition;
		this.desiredVelocity = desiredVelocity;
	}
}
