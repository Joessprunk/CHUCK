package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.instant;

import org.firstinspires.ftc.teamcode.Constants;

public class Catipult {

    private DcMotor leftArmMotor, rightArmMotor;
    private ElapsedTime catipultTimer = new ElapsedTime();


    public enum CatipultState{
        ONE_BALL,
        HALF_FULL,
        FULL
    }

    CatipultState catipultState;




    public void init(HardwareMap hwmap) {

        catipultState = CatipultState.FULL;


        leftArmMotor = hwmap.get(DcMotor.class, "left_arm");
        rightArmMotor = hwmap.get(DcMotor.class, "right_arm");

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        leftArmMotor.setTargetPosition(leftArmMotor.getCurrentPosition());
//        rightArmMotor.setTargetPosition(rightArmMotor.getCurrentPosition());

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     }


    public void shootCatipult(){
        catipultTimer.reset();
    }

    public Command shootCommand(){
        return instant(() -> shootCatipult());
    }



    public double calculateShootPower(){
        if(catipultState == CatipultState.ONE_BALL){
            return Constants.ONE_BALL_POWER;
        } else if (catipultState == CatipultState.HALF_FULL) {
            return Constants.HALF_POWER;
        }else {
            return Constants.FULL_POWER;
        }

    }

    public void setPower(double power){
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

    public void setCatipultState(CatipultState targetState){
        catipultState = targetState;
    }

    public CatipultState getCatipultState() {
        return catipultState;
    }

    public void periodic(){
        if(catipultTimer.seconds() <= 0.5){
            setPower(calculateShootPower());
        } else if (catipultTimer.seconds() > 1.0) {
            setPower(0.0);
        }else {
            setPower(-1.0);
        }
    }

//    public int getArmPosition(){
//        return (leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition())/2;
//    }


}




