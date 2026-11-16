package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

@Autonomous(name = "BLUE_SHOOT_GARDEN_DETECT", group = "Autonomous")
public class BLUE_SHOOT_GARDEN_DETECT extends LinearOpMode {

    private Intake intake = new Intake();
    private Catipult catipult = new Catipult();
    private VisionSys visionSys = new VisionSys();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startpose = poseFactory.of(57.3029, 9.5635, 270);
    private final Pose cornerIntake = poseFactory.of(9.25, 13.5, 266);
    private final Pose cornerIntakeControl1 = poseFactory.of(9.326, 53.1352, 0);
    private final Pose cornerShootStart = poseFactory.of(9.25, 13.5, 267);
    private final Pose cornerShoot = poseFactory.of(57.6727, 9.4616, 270);
    private final Pose cornerShootControl1 = poseFactory.of(30.488, 41.9705, 0);
    private final Pose flowerIntake = poseFactory.of(15.434, 45.9938, 180);
    private final Pose flowerIntakeControl1 = poseFactory.of(59.85, 30.7607, 0);
    private final Pose flowerIntakeControl2 = poseFactory.of(19.1725, 17.1748, 0);
    private final Pose flowerIntakeControl3 = poseFactory.of(60.7717, 47.0304, 0);
    private final Pose flowerIntakeSegment1Start = poseFactory.of(15.434, 45.9938, 270);
    private final Pose flowerIntakeSegment1End = poseFactory.of(15.434, 45.9938, 180);
    private final Pose flowerIntakeSegment2Heading = poseFactory.of(15.434, 45.9938, 180);
    private final Pose flowerShoot = poseFactory.of(57.9207, 9.3291, -90);
    private final Pose flowerShootControl1 = poseFactory.of(44.9138, 45.1497, 0);
    private final Pose flowerShootControl2 = poseFactory.of(28.4227, 10.3996, 0);
    private final Pose flowerShootSegment1Start = poseFactory.of(57.9207, 9.3291, 180);
    private final Pose flowerShootSegment1End = poseFactory.of(57.9207, 9.3291, 270);
    private final Pose flowerShootSegment2Heading = poseFactory.of(57.9207, 9.3291, -90);
    private final Pose point5 = poseFactory.of(9.9879, 90.8272, -82.924);
    private final Pose point5Control1 = poseFactory.of(15.5188, 46.2705, 0);
    private final Pose point5Segment1Start = poseFactory.of(9.9879, 90.8272, 270);
    private final Pose point5Segment1End = poseFactory.of(9.9879, 90.8272, 300);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                catipult.shootCommand(),
                waitMs(750),
                //intake.startIntakeCommand(),
                intake.startFlowerIntakeCommand(),
                race(
                        sequential(
                                follow(follower, cornerIntake()),
                                waitMs(1500)),
                        waitMs(4000)
                ),
                parallel(follow(follower, cornerShoot()),
                        sequential(waitMs(750),
                                intake.stopIntakeCommand())),
                race(
                        waitMs(20000), //HOW LONG TO WAIT BEFORE ENDING PATH AND GOING TO PARK
                        sequential(
                                visionSys.seesTagCommand(43),
                                waitMs(750),
                                catipult.shootCommand(),
                                waitMs(750),
                                parallel(
                                        follow(follower, flowerIntake()),
                                        sequential(
                                                intake.reverseIntakeCommand(),
                                                waitMs(500),
                                                intake.startFlowerIntakeCommand()
                                        )
                                ),
                                waitMs(1600),
                                parallel(
                                        follow(follower, flowerShoot()),
                                        sequential(
                                                waitMs(400),
                                                intake.startIntakeCommand(),
                                                waitMs(500),
                                                intake.stopIntakeCommand()
                                        )
                                ),
                                waitMs(500),
                                catipult.shootCommand(),
                                waitMs(750)
                        )
                ),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        catipult.init(hardwareMap);
        intake.init(hardwareMap);
        visionSys.init(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startpose);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            catipult.periodic();
            visionSys.periodic();
            follower.update();
            Scheduler.execute();

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

    public Path cornerShoot() {
        return Paths.curve(cornerShootStart, cornerShootControl1, cornerShoot).linear(cornerShootStart, cornerShoot);
    }

    public Path flowerIntake() {
        return Paths.curve(cornerShoot, flowerIntakeControl1, flowerIntakeControl2, flowerIntakeControl3, flowerIntake).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(flowerIntakeSegment1Start, flowerIntakeSegment1End)).until(1, Interpolator.constant(flowerIntakeSegment2Heading)));
    }

    public Path flowerShoot() {
        return Paths.curve(flowerIntake, flowerShootControl1, flowerShootControl2, flowerShoot).heading(Interpolator.piecewise().until(0.5931, Interpolator.linear(flowerShootSegment1Start, flowerShootSegment1End)).until(1, Interpolator.constant(flowerShootSegment2Heading)));
    }

    public Path path5() {
        return Paths.curve(flowerShoot, point5Control1, point5).heading(Interpolator.piecewise().until(0.1774, Interpolator.linear(point5Segment1Start, point5Segment1End)).until(1, Interpolator.tangent.reverse()));
    }
}
