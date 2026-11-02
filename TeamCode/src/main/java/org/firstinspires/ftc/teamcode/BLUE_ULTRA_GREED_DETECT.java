package org.firstinspires.ftc.teamcode;

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

import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

@Autonomous(name = "BLUE_ULTRA_GREED_DETECT", group = "Autonomous")
public class BLUE_ULTRA_GREED_DETECT extends LinearOpMode {

    private Intake intake = new Intake();
    private Catipult catipult = new Catipult();
    private VisionSys visionSys = new VisionSys();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose startpose = poseFactory.of(57.3029, 9.5635, 270);
    private final Pose cornerIntake = poseFactory.of(6.3971, 11.25, 267);
    private final Pose cornerIntakeControl1 = poseFactory.of(30.7495, 41.7432, 0);
    private final Pose cornerShootStart = poseFactory.of(6.3971, 15, 267);
    private final Pose cornerShoot = poseFactory.of(57.6727, 9.4616, 270);
    private final Pose cornerShootControl1 = poseFactory.of(30.488, 41.9705, 0);
    private final Pose insideCornerShootStart = poseFactory.of(57.6727, 9.4616, 270);
    private final Pose insideCornerShoot = poseFactory.of(65.1694, 125.9807, 86);
    private final Pose insideCornerShootControl1 = poseFactory.of(68.9411, 75.4687, 0);
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
    private final Pose park = poseFactory.of(12.1133, 102.43, 0);
    private final Pose parkControl1 = poseFactory.of(49.4816, 99.2808, 0);
    private final Pose flwShoot25 = poseFactory.of(57.4457, 9.0902, -90);
    private final Pose flwShoot25Control1 = poseFactory.of(41.0847, 39.7063, 0);
    private final Pose flwShoot25Control2 = poseFactory.of(28.8269, 11.7615, 0);
    private final Pose flwShoot25Segment1Start = poseFactory.of(57.4457, 9.0902, 180);
    private final Pose flwShoot25Segment1End = poseFactory.of(57.4457, 9.0902, 181);
    private final Pose flwShoot25Segment2Start = poseFactory.of(57.4457, 9.0902, -179);
    private final Pose flwShoot25Segment2End = poseFactory.of(57.4457, 9.0902, 270);
    private final Pose park25 = poseFactory.of(12.1133, 102.43, 99.801);
    private final Pose park25Control1 = poseFactory.of(23.9438, 33.0912, 0);
    private final Pose park25Segment1Start = poseFactory.of(12.1133, 102.43, 270);
    private final Pose park25Segment1End = poseFactory.of(12.1133, 102.43, 135);




    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                intake.stopIntakeCommand(),
                catipult.shootCommand(),
                waitMs(750),
                intake.startIntakeCommand(),
                race(
                        sequential(
                                follow(follower, cornerIntake()),
                                waitMs(1000)),
                        waitMs(4000)
                ),
                parallel(follow(follower, cornerShoot()),
                        sequential(waitMs(1000),
                                intake.stopIntakeCommand())),
                waitMs(750),
                conditional( () -> ( visionSys.seesTag(43) || visionSys.seesTag(44)),
                        sequential(
                                catipult.shootCommand(),
                                waitMs(750),
                                parallel(follow(follower, insideCornerShoot()),
                                        sequential(
                                                intake.reverseIntakeCommand(),
                                                waitMs(1000),
                                                intake.stopIntakeCommand()
                                        ))
                        ),
                        sequential(
                                follow(follower, insideCornerShoot()),
                                waitMs(500),
                                catipult.shootCommand(),
                                waitMs(750)
                        )

                ),
                parallel(
                        follow(follower, flwIntake()),
                        sequential(
                                intake.reverseIntakeCommand(),
                                waitMs(400),
                                intake.startFlowerIntakeCommand())),
                waitMs(1500),
                parallel(
                        follow(follower, flwShoot()),
                        sequential(
                                waitMs(300),
                                intake.startIntakeCommand(),
                                waitMs(750),
                                intake.stopIntakeCommand()
                        )),
                waitMs(500),
                catipult.shootCommand(),
                waitMs(750),
                parallel(
                        follow(follower, flwIntake2()),
                        sequential(
                                intake.reverseIntakeCommand(),
                                waitMs(1000),
                                intake.startFlowerIntakeCommand()
                        )
                ),
                waitMs(1500),
                conditional((()-> (visionSys.seesTag(42) || visionSys.seesTag(43))),
                        sequential(parallel(
                                        follow(follower, flwShoot25()),
                                        sequential(
                                                waitMs(500),
                                                intake.startIntakeCommand(),
                                                waitMs(500),
                                                intake.stopIntakeCommand()
                                        )
                                ),
                                waitMs(750),
                                conditional(()-> (visionSys.seesTag(42) || visionSys.seesTag(43)),
                                        sequential(
                                                catipult.shootCommand(),
                                                waitMs(750)),
                                        intake.stopIntakeCommand()
                                ),
                                follow(follower, park25()),
                                intake.stopIntakeCommand()
                        ),
                        sequential(parallel(
                                        follow(follower, flwShoot2()),
                                        sequential(
                                                waitMs(500),
                                                intake.startIntakeCommand(),
                                                waitMs(500),
                                                intake.stopIntakeCommand()
                                        )
                                ),
                                waitMs(750),
                                conditional(()-> (visionSys.seesTag(39) || visionSys.seesTag(40)),
                                        sequential(
                                                catipult.shootCommand(),
                                                waitMs(750)),
                                        intake.stopIntakeCommand()
                                ),
                                follow(follower, park()),
                                intake.stopIntakeCommand()
                        )
                )



        );
    }

    @Override
    public void runOpMode() {

        catipult.init(hardwareMap);
        intake.init(hardwareMap);
        visionSys.init(hardwareMap);

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
            visionSys.periodic();

        }
    }

    public Path cornerIntake() {
        return Paths.curve(startpose, cornerIntakeControl1, cornerIntake).linear(startpose, cornerIntake);
    }

    public Path cornerShoot() {
        return Paths.curve(cornerShootStart, cornerShootControl1, cornerShoot).linear(cornerShootStart, cornerShoot);
    }

    public Path insideCornerShoot() {
        return Paths.curve(insideCornerShootStart, insideCornerShootControl1, insideCornerShoot).linear(insideCornerShootStart, insideCornerShoot);
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
    public Path flwShoot25() {
        return Paths.curve(flwIntake2, flwShoot25Control1, flwShoot25Control2, flwShoot25).heading(Interpolator.piecewise().until(0.1613, Interpolator.linear(flwShoot25Segment1Start, flwShoot25Segment1End)).until(1, Interpolator.linear(flwShoot25Segment2Start, flwShoot25Segment2End)));
    }

    public Path park25() {
        return Paths.curve(flwShoot25, park25Control1, park25).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(park25Segment1Start, park25Segment1End)).until(1, Interpolator.tangent));
    }
}