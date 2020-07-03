package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.util.List;

public class Hardware {
    HardwareMap hardwareMap;
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public double angle;
    List<LynxModule> allHubs;
    private double centerWheelOffset = 0.182;
    public static double odoWidth =7.5645;
    private double portOffset = 0; // ticks
    private double starboardOffset = 0; //ticks
    public long xPosTicks = 0;
    public long yPosTicks = 0;
    public double xPosInches = 0;
    public double yPosInches = 0;
    private int previousPortReading;
    private int previousStarboardReading;
    private int previousLateralReading;
    private double previousAngleReading;
    private double prevW;
    private double prevLocalXVelo;
    private double prevLocalYVelo;
    public static double ticks_per_rotation = 8192;
    //38mm diameter, converted to inches = 1.4960630
    public static double circumfrence = 1.4960630 * Math.PI;
    public int ticker = 0;
    public int ticker2 = 0;
    public Motor[] hub1Motors;
    public Motor[] hub2Motors;
    public RegServo[] servos;
    public MecanumDrive mecanumDrive;
    public SixWheelDrive sixWheelDrive;
    public Hardware(HardwareMap hardwareMap){
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
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        hub1Motors = new Motor[4];//initialize here
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
    }

    public void loop(){
            allHubs.get(0).clearBulkCache(); // depends on which one is the odo hub
            int portReading=0;
            int starboardReading=0;
            int lateralReading=0;
            int PortChange = portReading - previousPortReading;
            int StarboardChange = starboardReading - previousStarboardReading;
            double deltaAngle;
            if(ticker % 3 == 0){
                angle = Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                deltaAngle = MathFunctions.keepAngleWithin180Degrees(angle - previousAngleReading);
                ticker = 1;
            }
            else{
                deltaAngle = (StarboardChange - PortChange) / (odoWidth* ticks_per_rotation / circumfrence);
                angle+=deltaAngle;
                ticker++;
            }
            double localX = lateralReading - previousLateralReading -(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));
            double localY = (PortChange + (deltaAngle * portOffset * (ticks_per_rotation/circumfrence)) + StarboardChange - (deltaAngle * starboardOffset * (ticks_per_rotation/circumfrence)))/2.0;
            if(deltaAngle < 0.00001){
                yPosTicks += (localY) * Math.sin(previousAngleReading + deltaAngle * 0.5) + (localX) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicks += (localY) * Math.cos(previousAngleReading + deltaAngle * 0.5) - (localX) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
            }
            else{
                xPosTicks += localY / deltaAngle * (Math.sin(angle)-Math.sin(previousAngleReading)) - localX/deltaAngle * (Math.cos(previousAngleReading)-Math.cos(angle));
                yPosTicks += localY / deltaAngle * (Math.cos(previousAngleReading) - Math.cos(angle)) + localX/deltaAngle * (Math.sin(angle) - Math.sin(previousAngleReading));
            }
            /*
            double portVelocity=0;
            double starboardVelocity=0;
            double lateralVelocity =0;
            double w = (starboardVelocity-portVelocity)/odoWidth;
            double localXVelocity = lateralVelocity - (w * centerWheelOffset * (ticks_per_rotation/circumfrence));
            double localYVelocity = (portVelocity + (w*portOffset * (ticks_per_rotation/circumfrence)) + starboardVelocity - (w * starboardOffset * (ticks_per_rotation/circumfrence)))/2.0;
            double Twy = 2*(localY)/(prevLocalYVelo+localYVelocity);
            double Twx = 2*(localX)/(prevLocalXVelo+localXVelocity);
            double Tg = 2*(deltaAngle)/(w + prevW);
            double angularAccel = (w - prevW)/Tg;
            long ClocalYf = OdoMath.C(0.56419*(Twy * prevW + Tg * angularAccel*Tg)/(Twy*Math.sqrt(angularAccel)));
            long Ci = OdoMath.C(0.56419*(prevW)/(Math.sqrt(angularAccel)));
            long SlocalYf = OdoMath.S(0.56419*(Twy * prevW + Tg * angularAccel*Tg)/(Twy*Math.sqrt(angularAccel)));
            long Si = OdoMath.S(0.56419*(prevW)/(Math.sqrt(angularAccel)));
            long ClocalXf = OdoMath.C(0.56419*(Twx * prevW + Tg * angularAccel*Tg)/(Twx*Math.sqrt(angularAccel)));
            long SlocalXf = OdoMath.S(0.56419*(Twx * prevW + Tg * angularAccel*Tg)/(Twx*Math.sqrt(angularAccel)));
            double globalYchangeForLocalYChange = prevLocalYVelo*1.77245*Twy/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*ClocalYf + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * SlocalYf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*Ci + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * Si));
            double globalXchangeForLocalYChange = prevLocalYVelo*1.77245*Twy/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*SlocalYf - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * ClocalYf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading)*Si - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-previousAngleReading) * Ci));
            double globalYchangeForLocalXChange = prevLocalXVelo*1.77245*Twx/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*ClocalXf + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * SlocalXf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*Ci + Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * Si));
            double globalXchangeForLocalXChange = prevLocalXVelo*1.77245*Twx/(Tg * Math.sqrt(angularAccel))*((Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*SlocalXf - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * ClocalXf) -(Math.cos(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90)))*Si - Math.sin(0.5*Math.pow(prevW,2)/angularAccel-(previousAngleReading-Math.toRadians(90))) * Ci));
            prevW = w;
            prevLocalXVelo = localXVelocity;
            prevLocalYVelo = localYVelocity;
            previousPortReading = portReading;
            previousStarboardReading = starboardReading;
            previousLateralReading = (int)localX;
            previousAngleReading = angle;

             */
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
    public double getX(){
        xPosInches = (double)xPosTicks * ticks_per_rotation / circumfrence;
        return xPosInches;
    }
    public double getY(){
        yPosInches = (double)yPosTicks * ticks_per_rotation / circumfrence;
        return yPosInches;
    }

}
