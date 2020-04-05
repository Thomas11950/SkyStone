package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LynxModuleMeta;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;

public class HardwareControllerThreadInterface extends Thread {
    HardwareMap hardwareMap;
    DcMotor Left;
    public boolean leftWriteRequested=false;
    public double leftWritePower;
    DcMotor Right;
    public boolean rightWriteRequested=false;
    public double rightWritePower;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public double angle;
    LinearOpMode parentOP;
    List<LynxModule> allHubs;
    private double centerWheelOffset = 0.182;
    private double starboardAndPortOffset=7.5645;
    public double xPosTicks = 0;
    public double yPosTicks = 0;
    public double xPosInches = 0;
    public double yPosInches = 0;
    private int previousPortReading;
    private int previousStarboardReading;
    private int previousLateralReading;
    private double previousAngleReading;
    public static double ticks_per_rotation = 8000 / 3.0;
    //38mm diameter, converted to inches = 1.4960630
    public static double circumfrence = 1.4960630 * Math.PI;
    public int ticker = 0;
    public HardwareControllerThreadInterface(HardwareMap hardwareMap, LinearOpMode parentOP){
        this.hardwareMap = hardwareMap;
        Left = this.hardwareMap.get(DcMotor.class,"left");
        Right = this.hardwareMap.get(DcMotor.class,"right");
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        previousAngleReading = angle;
        previousLateralReading = 0;
        previousStarboardReading = 0;
        previousPortReading = 0;
        this.parentOP = parentOP;
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public double keepAngleWithin180Degrees(double angle){
        while(angle > Math.toRadians(180)){
            angle -= Math.toRadians(360);
        }
        while(angle < Math.toRadians(180)){
            angle += Math.toRadians(360);
        }
        return angle;
    }
    public void run(){
        while(!parentOP.isStopRequested()){
            for(LynxModule module: allHubs){
                module.clearBulkCache();
            }
            int portReading=0;
            int starboardReading=0;
            int lateralReading=0;
            int PortChange = portReading - previousPortReading;
            int StarboardChange = starboardReading - previousStarboardReading;
            double localX = lateralReading - previousLateralReading;
            double deltaAngle;
            if(ticker % 3 == 0){
                angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                deltaAngle = keepAngleWithin180Degrees(angle - previousAngleReading);
                ticker = 1;
            }
            else{
                deltaAngle = (StarboardChange - PortChange) / (2 * starboardAndPortOffset* ticks_per_rotation / circumfrence);
                angle+=deltaAngle;
                ticker++;
            }
            double localY = (PortChange + StarboardChange)/2.0;
            if(deltaAngle < 0.00001){
                yPosTicks += (localY) * Math.sin(previousAngleReading + deltaAngle * 0.5) + (localX) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicks += (localY) * Math.cos(previousAngleReading + deltaAngle * 0.5) - (localX) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
            }
            else{
                xPosTicks += localY / deltaAngle * (Math.sin(angle)-Math.sin(previousAngleReading)) - localX/deltaAngle * (Math.cos(previousAngleReading)-Math.cos(angle));
                yPosTicks += localY / deltaAngle * (Math.cos(previousAngleReading) - Math.cos(angle)) + localX/deltaAngle * (Math.sin(angle) - Math.sin(previousAngleReading));
            }
            previousPortReading = portReading;
            previousStarboardReading = starboardReading;
            previousLateralReading = (int)localX;
            previousAngleReading = angle;
            if(rightWriteRequested){
                Right.setPower(rightWritePower);
                rightWriteRequested = false;
            }
            if(leftWriteRequested){
                Left.setPower(leftWritePower);
                leftWriteRequested = false;
            }
        }
    }
    public double getX(){
        xPosInches = xPosTicks * ticks_per_rotation / circumfrence;
        return xPosInches;
    }
    public double getY(){
        yPosInches = yPosTicks * ticks_per_rotation / circumfrence;
        return yPosInches;
    }
    public void writeRight(double power){
        rightWriteRequested = true;
        rightWritePower = power;
    }
    public void writeLeft(double power){
        leftWriteRequested = true;
        leftWritePower = power;
    }

}
