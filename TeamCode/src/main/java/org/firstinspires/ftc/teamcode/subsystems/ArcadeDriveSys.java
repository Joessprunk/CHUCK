package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArcadeDriveSys {

    private DcMotor RR_motor, FR_motor, FL_motor, RL_motor;


    private IMU imu;


    public void init(HardwareMap hwmap){

        // initializing the drive chassis motors with the Driver Hub Config. file
        FR_motor = hwmap.get(DcMotor.class, "FR_drive_motor"); //port 0
        RR_motor = hwmap.get(DcMotor.class, "RR_drive_motor"); //port 1
        FL_motor = hwmap.get(DcMotor.class, "FL_drive_motor"); //port 2
        RL_motor = hwmap.get(DcMotor.class, "RL_drive_motor"); //port 3



        FL_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL_motor.setDirection(DcMotor.Direction.REVERSE);
        RL_motor.setDirection(DcMotor.Direction.FORWARD);
        RR_motor.setDirection(DcMotor.Direction.FORWARD);
        FR_motor.setDirection(DcMotor.Direction.FORWARD);

        FL_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // initializing the IMU (Internal gyro)
        imu = hwmap.get(IMU.class, "imu");

        RevHubOrientationOnRobot HubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(HubOrientation));



    }

    public void autoInit(HardwareMap hwmap){

        // initializing the drive chassis motors with the Driver Hub Config. file
        FR_motor = hwmap.get(DcMotor.class, "FR_drive_motor"); //port 0
        RR_motor = hwmap.get(DcMotor.class, "RR_drive_motor"); //port 1
        FL_motor = hwmap.get(DcMotor.class, "FL_drive_motor"); //port 2
        RL_motor = hwmap.get(DcMotor.class, "RL_drive_motor"); //port 3

        FL_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL_motor.setTargetPosition(0);
        FR_motor.setTargetPosition(0);
        RR_motor.setTargetPosition(0);
        RL_motor.setTargetPosition(0);

        FL_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RL_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL_motor.setDirection(DcMotor.Direction.REVERSE);
        RL_motor.setDirection(DcMotor.Direction.FORWARD);
        RR_motor.setDirection(DcMotor.Direction.FORWARD);
        FR_motor.setDirection(DcMotor.Direction.FORWARD);

        FL_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // initializing the IMU (Internal gyro)
        imu = hwmap.get(IMU.class, "imu");

        RevHubOrientationOnRobot HubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(HubOrientation));


    }

    public void RobotOrientedDrive(double foward, double strafe, double rotate){
        double FL_Power = foward + strafe + rotate;
        double RL_Power = foward - strafe + rotate;
        double FR_Power = foward - strafe - rotate;
        double RR_Power = foward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(FL_Power));
        maxPower = Math.max(maxPower, Math.abs(RL_Power));
        maxPower = Math.max(maxPower, Math.abs(RR_Power));
        maxPower = Math.max(maxPower, Math.abs(FR_Power));

        FL_motor.setPower(maxSpeed * (FL_Power / maxPower));
        RL_motor.setPower(maxSpeed * (RL_Power / maxPower));
        RR_motor.setPower(maxSpeed * (RR_Power / maxPower));
        FR_motor.setPower(maxSpeed * (FR_Power / maxPower));


    }

    public void FieldOrientedDrive(double foward, double strafe, double rotate){
        double theta = Math.atan2(foward, strafe);
        double r = Math.hypot(strafe, foward);


        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newFoward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.RobotOrientedDrive(newFoward, newStrafe, rotate);
//        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

    }

    public double getTurnPowerToHeading(double targetAngle) {
        double currentAngle = getYaw();
        double error = targetAngle - currentAngle;

        // Normalize error to -180..180
        error = (error + 540) % 360 - 180;

        // Proportional control
        double kP = 0.01; // Tune this
        double turnPower = kP * error;

        // Clamp power
        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
//        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        return turnPower;
    }

    public double limeLightAim(double Tx) {
        double error = Tx;

        double kP = 0.03; // Tune this
        double turnPower = -kP * error;

        // Clamp power
        turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
//        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        return turnPower;
    }





    public void ResetPose(){
        imu.resetYaw();
//        pinpoint.resetPosAndIMU();
    }
    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
//    public double getPinpointYaw(){
//        return pinpoint.getHeading(AngleUnit.DEGREES);
//    }

    public boolean isBusy() {
        return FL_motor.isBusy() || FR_motor.isBusy() || RR_motor.isBusy() || RL_motor.isBusy();
    }

}


