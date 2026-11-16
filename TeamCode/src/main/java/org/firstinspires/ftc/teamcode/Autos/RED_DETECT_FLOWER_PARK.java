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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Catipult;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

@Autonomous(name = "RED_DETECT_FLOWER_PARK", group = "Autonomous")
public class RED_DETECT_FLOWER_PARK extends LinearOpMode {

    private Intake intake = new Intake();
    private Catipult catipult = new Catipult();
    private VisionSys visionSys = new VisionSys();

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();



    private final Pose start = poseFactory.of(59.3442, 131.6296, 90);
    private final Pose flowerIntake = poseFactory.of(46.8035, 127.7555, 90);
    private final Pose flowerIntakeControl1 = poseFactory.of(48.4892, 75.8071, 0);
    private final Pose jiggleOut = poseFactory.of(46.8, 120, 89.9738);
    private final Pose jiggleIn = poseFactory.of(46.8, 127.5, 90);
    private final Pose flowerShootStart = poseFactory.of(46.8, 127.5, 90);
    private final Pose flowerShoot = poseFactory.of(59.4232, 131.0538, 88);
    private final Pose flowerShootControl1 = poseFactory.of(54.7809, 75.1814, 0);
    private final Pose parkStart = poseFactory.of(59.4232, 131.0538, 88);
    private final Pose park = poseFactory.of(10.1352, 120.2195, 0);
    private final Pose parkControl1 = poseFactory.of(58.7042, 112.7787, 0);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                visionSys.seesTagCommand(32),
                waitMs(400),
                catipult.shootCommand(),
                waitMs(750),
                parallel(
                        follow(follower, flowerIntake()),
                        sequential(
                                waitMs(50),
                                intake.startFlowerIntakeCommand()
                        )),
                waitMs(1500),
                follow(follower, jiggleOut()),
                race(
                        sequential(
                                follow(follower, jiggleIn()),
                                waitMs(1000)
                        ),
                        waitMs(2500)
                ),
                parallel(
                        follow(follower, flowerShoot()),
                        sequential(
                                waitMs(400),
                                intake.stopIntakeCommand()
                        )),
                follow(follower, flowerShoot()),
                waitMs(750),
                catipult.shootCommand(),
                waitMs(750),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        catipult.init(hardwareMap);
        intake.init(hardwareMap);
        visionSys.init(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
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

    public Path flowerIntake() {
        return Paths.curve(start, flowerIntakeControl1, flowerIntake).linear(start, flowerIntake);
    }

    public Path jiggleOut() {
        return Paths.line(flowerIntake, jiggleOut).reverseTangent();
    }

    public Path jiggleIn() {
        return Paths.line(jiggleOut, jiggleIn).tangent();
    }

    public Path flowerShoot() {
        return Paths.curve(flowerShootStart, flowerShootControl1, flowerShoot).linear(flowerShootStart, flowerShoot);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
