package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "RED_GREED", group = "Autonomous")
public class RED_GREED extends LinearOpMode {

    private Intake intake = new Intake();
    private Catipult catipult = new Catipult();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose startpose = poseFactory.of(57.3029, 9.5635, 270);
    private final Pose cornerIntake = poseFactory.of(6.3971, 11.25, 267);
    private final Pose cornerIntakeControl1 = poseFactory.of(30.7495, 41.7432, 0);
    private final Pose outcrossShootStart = poseFactory.of(6.3971, 11.25, 270);
    private final Pose outcrossShoot = poseFactory.of(65.1694, 125.9807, 86);
    private final Pose outcrossShootControl1 = poseFactory.of(44.4457, 21.2661, 0);
    private final Pose outcrossShootControl2 = poseFactory.of(2.7192, 135.4632, 0);
    private final Pose outcrossShootControl3 = poseFactory.of(55.407, 86.2606, 0);
    private final Pose flwIntakeStart = poseFactory.of(65.1694, 125.9807, 90);
    private final Pose flwIntake = poseFactory.of(50.5506, 122.209, 90);
    private final Pose flwIntakeControl1 = poseFactory.of(52.639, 83.1769, 0);
    private final Pose flwShootStart = poseFactory.of(50.5506, 122.209, 90);
    private final Pose flwShoot = poseFactory.of(64.4042, 126.395, 86);
    private final Pose flwShootControl1 = poseFactory.of(54.9512, 76.9512, 0);
    private final Pose flwIntake2 = poseFactory.of(15.5727, 46.7357, 180);
    private final Pose flwIntake2Control1 = poseFactory.of(68.9512, 97.616, 0);
    private final Pose flwIntake2Control2 = poseFactory.of(5.0184, 119.4797, 0);
    private final Pose flwIntake2Control3 = poseFactory.of(63.8692, 56.2505, 0);
    private final Pose flwIntake2Segment1Start = poseFactory.of(15.5727, 46.7357, 86);
    private final Pose flwIntake2Segment1End = poseFactory.of(15.5727, 46.7357, 178);
    private final Pose flwIntake2Segment2Start = poseFactory.of(15.5727, 46.7357, 178);
    private final Pose flwIntake2Segment2End = poseFactory.of(15.5727, 46.7357, 180);
    private final Pose flwShoot2Start = poseFactory.of(15.5727, 46.7357, 180);
    private final Pose flwShoot2 = poseFactory.of(64.4816, 125.5737, 86);
    private final Pose flwShoot2Control1 = poseFactory.of(64.3683, 55.8536, 0);
    private final Pose flwShoot2Control2 = poseFactory.of(4.9761, 118.9678, 0);
    private final Pose flwShoot2Control3 = poseFactory.of(69.2284, 97.2965, 0);
    private final Pose parkStart = poseFactory.of(64.4816, 125.5737, 86);
    private final Pose park = poseFactory.of(13.1556, 116.5018, 0);
    private final Pose parkControl1 = poseFactory.of(49.4816, 99.2808, 0);



    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                intake.stopIntakeCommand(),
                catipult.shootCommand(),
                waitMs(750),
                intake.startIntakeCommand(),
                follow(follower, cornerIntake()),
                waitMs(1250),
                follow(follower, outcrossShoot()),
                intake.stopIntakeCommand(),
                waitMs(1000),
                catipult.shootCommand(),
                waitMs(750),
                intake.startFlowerIntakeCommand(),
                follow(follower, flwIntake()),
                waitMs(1500),
                intake.startIntakeCommand(),
                follow(follower, flwShoot()),
                intake.stopIntakeCommand(),
                waitMs(100),
                catipult.shootCommand(),
                waitMs(750),
                intake.startFlowerIntakeCommand(),
                follow(follower, flwIntake2()),
                waitMs(1500),
                intake.startIntakeCommand(),
                follow(follower, flwShoot2()),
                waitMs(1000),
                catipult.shootCommand(),
                waitMs(750),
                follow(follower, park()),
                intake.stopIntakeCommand()

        );
    }

    @Override
    public void runOpMode() {

        catipult.init(hardwareMap);
        intake.init(hardwareMap);

        Scheduler.reset();
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        follower.setPose(startpose);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            catipult.periodic();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path cornerIntake() {
        return Paths.curve(startpose, cornerIntakeControl1, cornerIntake).linear(startpose, cornerIntake);
    }

    public Path outcrossShoot() {
        return Paths.curve(outcrossShootStart, outcrossShootControl1, outcrossShootControl2, outcrossShootControl3, outcrossShoot).linear(outcrossShootStart, outcrossShoot);
    }

    public Path flwIntake() {
        return Paths.curve(flwIntakeStart, flwIntakeControl1, flwIntake).linear(flwIntakeStart, flwIntake);
    }

    public Path flwShoot() {
        return Paths.curve(flwShootStart, flwShootControl1, flwShoot).linear(flwShootStart, flwShoot);
    }

    public Path flwIntake2() {
        return Paths.curve(flwShoot, flwIntake2Control1, flwIntake2Control2, flwIntake2Control3, flwIntake2).heading(Interpolator.piecewise().until(0.6814, Interpolator.linear(flwIntake2Segment1Start, flwIntake2Segment1End)).until(1, Interpolator.linear(flwIntake2Segment2Start, flwIntake2Segment2End)));
    }

    public Path flwShoot2() {
        return Paths.curve(flwShoot2Start, flwShoot2Control1, flwShoot2Control2, flwShoot2Control3, flwShoot2).linear(flwShoot2Start, flwShoot2);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}