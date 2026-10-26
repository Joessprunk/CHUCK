package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.instant;

import org.firstinspires.ftc.teamcode.Constants;

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
        setFlowerArm(Constants.FLOWER_ARM_UP);
    }
    public void startFlowerIntake(){
        intakeMotor.setPower(1.0);
        setFlowerArm(Constants.FLOWER_ARM_DOWN);
    }
    public void stopIntake(){
        intakeMotor.setPower(0);
        setFlowerArm(Constants.FLOWER_ARM_UP);
    }
    public void setFlowerArm(double pos){
        leftFlowerArm.setPosition(pos);
        rightFlowerArm.setPosition(pos);
    }

    public Command startIntakeCommand(){
        return instant(() -> startintake());
    }
    public Command startFlowerIntakeCommand(){
        return instant(() -> startFlowerIntake());
    }
    public Command stopIntakeCommand(){
        return instant(() -> stopIntake());
    }

    public void reverseIntake() {
        intakeMotor.setPower(-1.0);
    }
}
