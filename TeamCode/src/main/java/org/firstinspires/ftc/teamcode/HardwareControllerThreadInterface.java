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
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.util.List;

public class HardwareControllerThreadInterface extends Thread {
    HardwareMap hardwareMap;
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
    public int ticker2 = 0;
    public Motor[] hub1Motors;
    public Motor[] hub2Motors;
    public RegServo[] servos;
    public MecanumDrive mecanumDrive;
    public HardwareControllerThreadInterface(HardwareMap hardwareMap, LinearOpMode parentOP){
        this.hardwareMap = hardwareMap;
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
        hub1Motors = new Motor[4];//initialize here
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
    }

    public void run(){
        while(!parentOP.isStopRequested()){
            allHubs.get(0).clearBulkCache(); // depends on which one is the odo hub
            int portReading=0;
            int starboardReading=0;
            int lateralReading=0;
            int PortChange = portReading - previousPortReading;
            int StarboardChange = starboardReading - previousStarboardReading;
            double localX = lateralReading - previousLateralReading;
            double deltaAngle;
            if(ticker % 3 == 0){
                angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                deltaAngle = MathFunctions.keepAngleWithin180Degrees(angle - previousAngleReading);
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
            for(Motor motor: hub1Motors){
                if(motor.setTargetPosRequested){
                    motor.motor.setTargetPosition(motor.targetPosition);
                    motor.setTargetPosRequested = false;
                }
            }
            for(Motor motor: hub1Motors){
                if(motor.writePowerRequested){
                    motor.motor.setPower(motor.power);
                    motor.writePowerRequested = false;
                }
                if(motor.writeVelocityRequested){
                    motor.motor.setVelocity(motor.velocity);
                    motor.writeVelocityRequested = false;
                }
            }
            for(Motor motor: hub2Motors){
                if(motor.setTargetPosRequested){
                    motor.motor.setTargetPosition(motor.targetPosition);
                    motor.setTargetPosRequested = false;
                }
            }
            for(Motor motor: hub2Motors){
                if(motor.writePowerRequested){
                    motor.motor.setPower(motor.power);
                    motor.writePowerRequested = false;
                }
                if(motor.writeVelocityRequested){
                    motor.motor.setVelocity(motor.velocity);
                    motor.writeVelocityRequested = false;
                }
            }
            for(RegServo servo: servos){
                if(servo.writeRequested){
                    servo.servo.setPosition(servo.position);
                    servo.writeRequested = false;
                }
            }
            boolean hub2ReadNeeded = false;
            for(Motor motor: hub2Motors){
                if(motor.readRequested)
                    hub2ReadNeeded = true;
            }
            if(ticker2 % 10 == 0 && hub2ReadNeeded){
                allHubs.get(1).clearBulkCache(); // depends on which one is not the odo hub
                for(Motor motor: hub2Motors){
                    if(motor.readRequested){
                        motor.currentPosition = motor.motor.getCurrentPosition();
                    }
                }
                ticker2 = 1;
            }
            else if (hub2ReadNeeded){
                ticker2++;
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

}
