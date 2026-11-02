package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArcadeDriveSys;
import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;




@TeleOp
public class SoloOpMode extends OpMode {
    ArcadeDriveSys driveSys = new ArcadeDriveSys();
    Catipult catipult = new Catipult();
    Intake intake = new Intake();
    VisionSys visionSys = new VisionSys();
    //ElapsedTime catapultShootTimer = new ElapsedTime();

    TelemetryManager panelsTelemetry;


    double foward, rotate, strafe;

    @Override
    public void init() {
        intake.init(hardwareMap);
        driveSys.init(hardwareMap);
        catipult.init(hardwareMap);
        visionSys.init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void loop() {

        double robotAngle = driveSys.getYaw();

        telemetry.addData("robotYaw", driveSys.getYaw());

        if(gamepad1.a){
            if (visionSys.seesTag(35)) {
                rotate = visionSys.getTagAimRotation(35);
            }
            else if (visionSys.seesTag(32)) {
                rotate = visionSys.getTagAimRotation(32);
            }
            else {
                rotate = 0.0;
            }
        }
//        else if(gamepad1.y){
//            if(visionSys.seesTag(36) && visionSys.seesTag(35)){
//                rotate = visionSys.getTargetRotation(35, 36);
//            }
//            else if (visionSys.seesTag(32) && visionSys.seesTag(33)) {
//                rotate = visionSys.getTargetRotation(32, 33);
//            }
//      }
        else{
            rotate = -gamepad1.right_stick_x;
        }

        foward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;

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

//        if(gamepad1.x){
//            visionSys.setMode(VisionSys.VisionMode.APRIL_TAGS);
//        } else if (gamepad1.b) {
//            visionSys.setMode(VisionSys.VisionMode.GAME_PIECES);
//        }

        // Operator Controls for comp setup

        if(gamepad1.left_bumper){
            catipult.shootCatipult();
        }

        if (gamepad1.x){
            catipult.setCatipultState(Catipult.CatipultState.ONE_BALL);
        } else if (gamepad1.b) {
            catipult.setCatipultState(Catipult.CatipultState.HALF_FULL);
        } else if (gamepad1.y) {
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
        visionSys.periodic();
        catipult.periodic(); // required in every op mode to operate the state machine of the catapult. dont forget


        // TELEMETRY
//        telemetry.addData("Catapult Current State: ",catipult.getCatipultState());
//        telemetry.update();

        panelsTelemetry.addData("Robot Yaw", driveSys.getYaw());

        panelsTelemetry.addData("Catapult State",
                catipult.getCatipultState());

        panelsTelemetry.addData("Vision Mode",
                visionSys.getCurrentMode());

        panelsTelemetry.addData("AprilTag 35 Visible",
                visionSys.seesTag(35));

        panelsTelemetry.addData("AprilTag 36 Visible",
                visionSys.seesTag(36));

        panelsTelemetry.addData("Game Piece Visible",
                visionSys.seesGamePiece());

// Update Panels and Driver Station
        panelsTelemetry.update(telemetry);

    }

}

