package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Catipult {

    private DcMotor leftArmMotor, rightArmMotor;




    public void init(HardwareMap hwmap) {




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




    public void setPower(double power){
        leftArmMotor.setPower(power);
        rightArmMotor.setPower(power);
    }

//    public int getArmPosition(){
//        return (leftArmMotor.getCurrentPosition() + rightArmMotor.getCurrentPosition())/2;
//    }


}




