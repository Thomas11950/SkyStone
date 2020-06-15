package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.hardware.Motor;

public class SixWheelDriveBenjamin {
    Motor Left;
    Motor Right;
    public SixWheelDriveBenjamin (Motor Left, Motor Right){
        this.Left = Left;
        this.Right = Right;
    }
    public void setPowers(double forwardsBackPower, double turnPower){
        double sum = Math.abs(forwardsBackPower) + Math.abs(turnPower);
        double leftTurn = 0;
        double rightTurn = 0;
        if(sum > 1 || sum < -1) {
            if (turnPower > 0)
                leftTurn = turnPower / sum;
            if (turnPower < 0)
                rightTurn = turnPower / sum * -1;
            double leftForward = forwardsBackPower / sum;
            double rightForward = forwardsBackPower / sum;
            Left.setPower(leftTurn + leftForward);
            Right.setPower(rightTurn + rightForward);
        }
        else{
            if (turnPower > 0) {
                Left.setPower(forwardsBackPower + turnPower);
                Right.setPower(forwardsBackPower);
            }
            if (turnPower < 0){
                Left.setPower(forwardsBackPower);
                Right.setPower(forwardsBackPower + turnPower);
            }
        }
    }
}
