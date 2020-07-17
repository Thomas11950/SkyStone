package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
    public List<LynxModule> allHubs;
    private double centerWheelOffset = 3.847;
    public static double trackWidth = 16.037;
    private static double portOffset = 7.674; // inches
    private static double starboardOffset = 8.103; //inches
    public static double odoWidth =portOffset+starboardOffset;
    public double xPosTicks = 0;
    public double yPosTicks = 0;
    public double xPosInches = 0;
    public double yPosInches = 0;
    public double yPosTicksAlt=0;
    public double xPosTicksAlt=0;
    private int previousPortReading;
    private int previousStarboardReading;
    private int previousLateralReading;
    private double previousAngleReading;
    private double prevW;
    private double prevLocalXVelo;
    private double prevLocalYVelo;
    public static double ticks_per_rotation = 8192;
    //38mm diameter, converted to inches = 1.4960630
    public static double driveWheelRadius = 1.41732;
    public static double odoWheelRadius = 0.6889764;
    public static double circumfrence = odoWheelRadius * 2 * Math.PI / (92.7070970616/96);
    public int ticker = 0;
    public Motor[] hub1Motors;
    public Motor[] hub2Motors;
    public RegServo[] servos;
    public MecanumDrive mecanumDrive;
    public SixWheelDrive sixWheelDrive;
    public ElapsedTime time;
    public int loops = 0;
    public boolean firstLoop = true;
    public double startTime;
    public double prevTime;
    String TAG = "odo";
    public boolean updatePID;
    public Hardware(HardwareMap hardwareMap){
        updatePID = false;
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
        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        hub1Motors = new Motor[4];//initialize here
        hub1Motors[0] = new Motor( hardwareMap.get(DcMotorEx.class,"LF"));
        hub1Motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        hub1Motors[1] = new Motor(hardwareMap.get(DcMotorEx.class,"LB"));
        hub1Motors[1].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[2] = new Motor(hardwareMap.get(DcMotorEx.class,"RF"));
        hub1Motors[2].motor.setDirection(DcMotorEx.Direction.REVERSE);
        hub1Motors[3] = new Motor(hardwareMap.get(DcMotorEx.class,"RB"));
        hub1Motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        time = new ElapsedTime();
        sixWheelDrive = new SixWheelDrive(hub1Motors[0],hub1Motors[1],hub1Motors[2],hub1Motors[3],time);
        hub2Motors = new Motor[4];//initialize here
        servos = new RegServo[12];//initialize here
    }

    public void loop(){
        loops++;
        double currentTime = time.milliseconds();
        hub1Motors[0].readRequested = true;
        hub1Motors[1].readRequested = true;
        hub1Motors[3].readRequested = true;
        boolean hub1ReadNeeded = true;
        for(Motor motor: hub1Motors){
            if(motor != null && motor.readRequested){
                hub1ReadNeeded = true;
            }
        }
        allHubs.get(0).clearBulkCache();
        for(Motor motor: hub1Motors){
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }
        boolean hub2ReadNeeded = false;
        for(Motor motor: hub2Motors){
            if(motor != null && motor.readRequested)
                hub2ReadNeeded = true;
        }
        //`allHubs.get(1).clearBulkCache(); // depends on which one is not the odo hub
        for(Motor motor: hub2Motors){
            if(motor != null && motor.readRequested){
                motor.currentPosition = motor.motor.getCurrentPosition();
                motor.currentVelocity = motor.motor.getVelocity(AngleUnit.RADIANS);
            }
        }

            int portReading=hub1Motors[0].getCurrentPosition();
            int starboardReading=hub1Motors[3].getCurrentPosition();
            int lateralReading=-hub1Motors[1].getCurrentPosition();

        if(firstLoop){
            startTime = time.milliseconds();
            prevTime = startTime;
            previousLateralReading = lateralReading;
            previousStarboardReading = starboardReading;
            previousPortReading = portReading;
            firstLoop = false;
        }
        double portVelo=(portReading-previousPortReading)/(currentTime - prevTime) * circumfrence / ticks_per_rotation;//inches/sec
        double starboardVelo=(starboardReading-previousStarboardReading)/(currentTime-prevTime) * circumfrence / ticks_per_rotation;//inches/sec
            int PortChange = portReading - previousPortReading;
            int StarboardChange = starboardReading - previousStarboardReading;
            double deltaAngle;
            /*if(ticker % 3 == 0){*/
                angle = MathFunctions.keepAngleWithin180Degrees(Math.toRadians(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)+Math.toRadians(180));
                deltaAngle = MathFunctions.keepAngleWithin180Degrees(angle - previousAngleReading);
                ticker = 1;/*
            }
            else{
                deltaAngle = (StarboardChange - PortChange) / (odoWidth* ticks_per_rotation / circumfrence);
                angle+=deltaAngle;
                ticker++;
            }*/
            double localX = lateralReading - previousLateralReading;//-(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));
            double localXAlt =  lateralReading - previousLateralReading-(deltaAngle * centerWheelOffset * (ticks_per_rotation/circumfrence));

        /*double localY = (PortChange + (deltaAngle * portOffset * (ticks_per_rotation/circumfrence)) + StarboardChange - (deltaAngle * starboardOffset * (ticks_per_rotation/circumfrence)))/2.0;
 */         double localY = PortChange; //
            double localYAlt = PortChange+ (deltaAngle * portOffset * (ticks_per_rotation/circumfrence));
            RobotLog.dd(TAG, "lateralReading: " + lateralReading);
            RobotLog.dd(TAG, "prevlateral: " + previousLateralReading);
            RobotLog.dd(TAG, "portchange: " + PortChange);
            RobotLog.dd(TAG, "deltaAngle: " + deltaAngle+", prevAngle: " +previousAngleReading);
            RobotLog.dd(TAG, "localX: "+localX);
            RobotLog.dd(TAG,"localY: "+localY);
            if(deltaAngle < Math.toRadians(0.05)||localY==0||localX==0||true){
                yPosTicks += (localY) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localX) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicks += (localY) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localX) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
                yPosTicksAlt += (localYAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localXAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5);//updates Y position
                xPosTicksAlt += (localYAlt) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localXAlt) * Math.sin(previousAngleReading + deltaAngle * 0.5);//updates X position
                RobotLog.dd(TAG, "yPosTicks increment: "+ ((localY) * Math.sin(previousAngleReading + deltaAngle * 0.5) - (localX) * Math.cos(previousAngleReading + deltaAngle * 0.5)));//updates Y position)
                RobotLog.dd(TAG, "xPosTicks increment: " +((localY) * Math.cos(previousAngleReading + deltaAngle * 0.5) + (localX) * Math.sin(previousAngleReading + deltaAngle * 0.5)));//updates X position))
                RobotLog.dd(TAG, "xPosTicks: "+ xPosTicks);
                RobotLog.dd(TAG,"yPosTicks: "+yPosTicks);
            }
            else{
                xPosTicks += localY / deltaAngle * (Math.sin(angle)-Math.sin(previousAngleReading)) + localX/deltaAngle * (Math.cos(previousAngleReading)-Math.cos(angle));
                yPosTicks += localY / deltaAngle * (Math.cos(previousAngleReading) - Math.cos(angle)) - localX/deltaAngle * (Math.sin(angle) - Math.sin(previousAngleReading));
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
            double w = (starboardVelo - portVelo)/odoWidth;//inches
        double localYVelocity = (portVelo + (w*portOffset) + starboardVelo - (w * starboardOffset))/2.0;
        if(updatePID) {
            sixWheelDrive.updatePID(localYVelocity - w * trackWidth / 2, localYVelocity + w * trackWidth / 2);
        }
        for(Motor motor: hub1Motors){
                if(motor!=null&&motor.setTargetPosRequested){
                    motor.motor.setTargetPosition(motor.targetPosition);
                    motor.setTargetPosRequested = false;

                }
            }
            for(Motor motor: hub1Motors){
                if(motor!=null&&motor.writePowerRequested){
                    motor.motor.setPower(motor.power);
                    motor.writePowerRequested = false;
                }
                if(motor!=null&&motor.writeVelocityRequested){
                    motor.motor.setVelocity(motor.velocity);
                    motor.writeVelocityRequested = false;
                }
            }
            for(Motor motor: hub2Motors){
                if(motor!=null&&motor.setTargetPosRequested){
                    motor.motor.setTargetPosition(motor.targetPosition);
                    motor.setTargetPosRequested = false;
                }
            }
            for(Motor motor: hub2Motors){
                if(motor!=null&&motor.writePowerRequested){
                    motor.motor.setPower(motor.power);
                    motor.writePowerRequested = false;
                }
                if(motor!=null&&motor.writeVelocityRequested){
                    motor.motor.setVelocity(motor.velocity);
                    motor.writeVelocityRequested = false;
                }
            }
            for(RegServo servo: servos){
                if(servo!=null&&servo.writeRequested){
                    servo.servo.setPosition(servo.position);
                    servo.writeRequested = false;
                }
            }
            previousLateralReading = lateralReading;
            previousAngleReading = angle;
            previousPortReading = portReading;
            previousStarboardReading = starboardReading;
    }
    public double getX(){
        xPosInches = (double)xPosTicks * circumfrence / ticks_per_rotation;
        return xPosInches;
    }
    public double getY(){
        yPosInches = (double)yPosTicks * circumfrence / ticks_per_rotation;
        return yPosInches;
    }

}
