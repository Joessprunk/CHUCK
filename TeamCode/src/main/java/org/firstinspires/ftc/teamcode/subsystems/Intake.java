package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {


    private DcMotor intakeMotor;
    private Servo leftFlowerArm, rightFlowerArm;



    public void init(HardwareMap hwmap) {

        intakeMotor = hwmap.get(DcMotor.class, "intake");

        leftFlowerArm = hwmap.get(Servo.class, "left_flower");
        rightFlowerArm = hwmap.get(Servo.class, "right_flower");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFlowerArm.setDirection(Servo.Direction.FORWARD);
        rightFlowerArm.setDirection(Servo.Direction.REVERSE);

    }
    public void  startintake(){
        intakeMotor.setPower(1.0);
    }
    public void stopintake(){
        intakeMotor.setPower(0);
    }
    public void setFlowerArm(double pos){
        leftFlowerArm.setPosition(pos);
        rightFlowerArm.setPosition(pos);
    }
}
