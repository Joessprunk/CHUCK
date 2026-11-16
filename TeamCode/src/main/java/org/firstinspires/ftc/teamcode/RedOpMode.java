package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArcadeDriveSys;
import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LightSys;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;




@TeleOp
public class RedOpMode extends OpMode {
    ArcadeDriveSys driveSys = new ArcadeDriveSys();
    Catipult catipult = new Catipult();
    Intake intake = new Intake();
    VisionSys visionSys = new VisionSys();
    LightSys lightSys = new LightSys();
    //ElapsedTime catapultShootTimer = new ElapsedTime();

    TelemetryManager panelsTelemetry;


    double foward, rotate, strafe;

    @Override
    public void init() {
        intake.init(hardwareMap);
        driveSys.init(hardwareMap);
        catipult.init(hardwareMap);
        visionSys.init(hardwareMap);
        lightSys.init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void loop() {

        double robotAngle = driveSys.getYaw();

        telemetry.addData("robotYaw", driveSys.getYaw());

        if(gamepad1.left_bumper){
            if (visionSys.seesTag(36)) {
                rotate = visionSys.getTagAimRotation(36);
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


        if (gamepad1.left_bumper && !(visionSys.isTagOnTarget(36) || visionSys.isTagOnTarget(32))) {
            lightSys.setLightState(LightSys.LightState.aiming);
        } else if (gamepad1.left_bumper && (visionSys.isTagOnTarget(36) || visionSys.isTagOnTarget(32))) {
            lightSys.setLightState(LightSys.LightState.on_target);
        } else if((catipult.getCatapultState() == Catipult.CatipultState.FULL) && (gamepad1.right_trigger > 0.5 || gamepad1.right_bumper)){
            lightSys.setLightState(LightSys.LightState.FULL_intaking);
        }else if((catipult.getCatapultState() == Catipult.CatipultState.HALF_FULL) && (gamepad1.right_trigger > 0.5 || gamepad1.right_bumper)){
            lightSys.setLightState(LightSys.LightState.MID_intaking);
        }else if((catipult.getCatapultState() == Catipult.CatipultState.ONE_BALL) && (gamepad1.right_trigger > 0.5 || gamepad1.right_bumper)){
            lightSys.setLightState(LightSys.LightState.LOW_intaking);
        }else if(catipult.getCatapultState() == Catipult.CatipultState.FULL){
            lightSys.setLightState(LightSys.LightState.FULL_POWER);
        }else if(catipult.getCatapultState() == Catipult.CatipultState.HALF_FULL){
            lightSys.setLightState(LightSys.LightState.MID_POWER);
        }else if(catipult.getCatapultState() == Catipult.CatipultState.ONE_BALL){
            lightSys.setLightState(LightSys.LightState.LOW_POWER);
        }else {
            lightSys.setLightState(LightSys.LightState.MID_intaking);
        }


        lightSys.updateLights();
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

// Update Panels and Driver Station
        panelsTelemetry.update(telemetry);

    }

}

