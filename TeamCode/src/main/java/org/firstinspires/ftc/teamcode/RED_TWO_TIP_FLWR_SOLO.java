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

@Autonomous(name = "RED_TWO_TIP_FLWR", group = "Autonomous")
public class RED_TWO_TIP_FLWR_SOLO extends LinearOpMode {

    private Intake intake = new Intake();
    private Catipult catipult = new Catipult();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startpose = poseFactory.of(57.3029, 9.5635, 90);
    private final Pose cornerIntake = poseFactory.of(16.56, 44.8872, -180);
    private final Pose cornerIntakeControl1 = poseFactory.of(50.0331, 41.222, 0);
    private final Pose cornerIntakeSegment1Start = poseFactory.of(16.56, 44.8872, 270);
    private final Pose cornerIntakeSegment1End = poseFactory.of(16.56, 44.8872, 183);
    private final Pose cornerIntakeSegment2Start = poseFactory.of(16.56, 44.8872, -177);
    private final Pose cornerIntakeSegment2End = poseFactory.of(16.56, 44.8872, 180);
    private final Pose outcrossShootStart = poseFactory.of(16.56, 44.8872, 270);
    private final Pose outcrossShoot = poseFactory.of(65.1694, 125.9807, 86);
    private final Pose outcrossShootControl1 = poseFactory.of(33.2403, 82.244, 0);
    private final Pose outcrossShootControl2 = poseFactory.of(2.7192, 135.4632, 0);
    private final Pose outcrossShootControl3 = poseFactory.of(55.407, 86.2606, 0);
    private final Pose flwIntakeStart = poseFactory.of(65.1694, 125.9807, 90);
    private final Pose flwIntake = poseFactory.of(50.5506, 122.209, 90);
    private final Pose flwIntakeControl1 = poseFactory.of(52.639, 83.1769, 0);
    private final Pose flwShootStart = poseFactory.of(50.5506, 122.209, 90);
    private final Pose flwShoot = poseFactory.of(64.4042, 126.395, 86);
    private final Pose flwShootControl1 = poseFactory.of(53.6483, 84.2477, 0);
    private final Pose pARKKKKKStart = poseFactory.of(64.4042, 126.395, 90);
    private final Pose pARKKKKK = poseFactory.of(10.163, 116.2717, 0);
    private final Pose pARKKKKKControl1 = poseFactory.of(66.5083, 96.2449, 0);



    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                intake.stopIntakeCommand(),
                catipult.shootCommand(),
                waitMs(1000),
                intake.startIntakeCommand(),
                follow(follower, cornerIntake()),
                waitMs(1000),
                follow(follower, outcrossShoot()),
                intake.stopIntakeCommand(),
                waitMs(500),
                catipult.shootCommand(),
                waitMs(1000),
                intake.startFlowerIntakeCommand(),
                follow(follower, flwIntake()),
                waitMs(1500),
                intake.startIntakeCommand(),
                follow(follower, flwShoot()),
                intake.stopIntakeCommand(),
                waitMs(500),
                catipult.shootCommand(),
                waitMs(750),
                follow(follower, pARKKKKK())
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
        return Paths.curve(startpose, cornerIntakeControl1, cornerIntake).heading(Interpolator.piecewise().until(0.5451, Interpolator.linear(cornerIntakeSegment1Start, cornerIntakeSegment1End)).until(1, Interpolator.linear(cornerIntakeSegment2Start, cornerIntakeSegment2End)));
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

    public Path pARKKKKK() {
        return Paths.curve(pARKKKKKStart, pARKKKKKControl1, pARKKKKK).linear(pARKKKKKStart, pARKKKKK);
    }
}