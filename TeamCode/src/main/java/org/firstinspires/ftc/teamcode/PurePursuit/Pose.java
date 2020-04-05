package org.firstinspires.ftc.teamcode.PurePursuit;

public class Pose {
    public double X;
    public double Y;
    public double power;
    public double translationPower;
    public double rotationPower;
    public Pose(double X, double Y, double power, double translationPower, double rotationPower){
        this.X = X;
        this.Y = Y;
        this.power = power;
        this.translationPower = translationPower;
        this.rotationPower = rotationPower;
    }
}
