package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArcadeDriveSys;
import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class MainOpMode extends OpMode {
    ArcadeDriveSys driveSys = new ArcadeDriveSys();
    Catipult catipult = new Catipult();
    Intake intake = new Intake();
    //ElapsedTime catapultShootTimer = new ElapsedTime();


    boolean isShooting = false;

    double foward, rotate, strafe;

    @Override
    public void init() {
        intake.init(hardwareMap);
        driveSys.init(hardwareMap);
        catipult.init(hardwareMap);
    }

    public void loop() {

        double robotAngle = driveSys.getYaw();

        telemetry.addData("robotYaw", driveSys.getYaw());
        foward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        driveSys.FieldOrientedDrive(foward, strafe, rotate);

        // Driver controls for comp setup
        if (gamepad1.start) {
            driveSys.ResetPose();
        }

        if(gamepad1.right_bumper){
            intake.startFlowerIntake();
        } else if (gamepad1.right_trigger >= 0.3) {
            intake.startintake();
        } else if (gamepad1.left_trigger >= 0.5) {
            intake.reverseIntake();
        }else
        {
            intake.stopIntake();
        }

        // Operator Controls for comp setup

        if(gamepad2.right_trigger >= 0.7){
            catipult.shootCatipult();
        }

        if (gamepad2.a){
            catipult.setCatipultState(Catipult.CatipultState.ONE_BALL);
        } else if (gamepad2.b) {
            catipult.setCatipultState(Catipult.CatipultState.HALF_FULL);
        } else if (gamepad2.y) {
            catipult.setCatipultState(Catipult.CatipultState.FULL);
        }



//
//
//        if(gamepad1.right_trigger >= 0.5){
//            catipult.setPower(1.0);
//            catapultShootTimer.reset();
//        }else if(catapultShootTimer.seconds() < 0.35){
//            catipult.setPower(1.0);
//        }else if (catapultShootTimer.seconds() > 0.85) {
//            catipult.setPower(0.0);
//        }else if (catapultShootTimer.seconds() > 0.35){
//            catipult.setPower(-1.0);
//        }

        catipult.periodic(); // required in every op mode to operate the state machine of the catapult. dont forget


        // TELEMETRY
        telemetry.addData("Catapult Current State: ",catipult.getCatipultState());
        telemetry.update();

    }

}

