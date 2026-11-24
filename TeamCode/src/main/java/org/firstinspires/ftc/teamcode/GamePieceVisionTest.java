package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CollectionPlanner;
import org.firstinspires.ftc.teamcode.subsystems.GamePieceCollection;
import org.firstinspires.ftc.teamcode.subsystems.GamePieceTracker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

import java.util.List;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * TeleOp test: manual driving + tracked-ball telemetry + fixed-route variable 1..4-ball automatic pickup.
 *
 * A = clear tracker (manual mode only)
 * B = plan and collect UP TO FOUR balls (1..4 available)
 * X = cancel pickup and stop the follower/intake
 *
 * Uses a bounded, speed-limited straight-line route. No obstacle avoidance.
 */
@TeleOp(name = "Game Piece Collection Test", group = "Testing")
public class GamePieceVisionTest extends OpMode {

    private final VisionSys visionSys = new VisionSys();
    private final GamePieceTracker tracker = new GamePieceTracker();
    private final Intake intake = new Intake();

    // Allowed BALL field coordinates (inches).
    private static final double MIN_X = -3.0;
    private static final double MAX_X = 70.0;
    private static final double MIN_Y = -3.0;
    private static final double MAX_Y = 40.0;

    // Current experimental effective pickup offset from robot center (inches).
    private static final double PICKUP_FORWARD_IN = 6.0;

    private final CollectionPlanner planner = new CollectionPlanner(
            MIN_X, MAX_X, MIN_Y, MAX_Y, PICKUP_FORWARD_IN);

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(57.3029, 9.5635, 270);

    private Follower follower;
    private GamePieceCollection collection;

    // Tunable manual control response. Drive forward unchanged; strafe/turn reversed.
    private static final double MANUAL_DRIVE_SCALE = 0.40;

    // One continuous intake run wraps all planned pickup segments.
    private Command pickupCommand;
    private boolean pickupScheduled = false;

    // Rising-edge button tracking: one action per press.
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;

    private String testMessage = "Manual drive ready";

    @Override
    public void init() {
        Scheduler.reset();
        visionSys.init(hardwareMap);
        intake.init(hardwareMap);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        tracker.clear();

        collection = new GamePieceCollection(follower, visionSys, tracker, planner);

        telemetry.addLine("GAME PIECE COLLECTION TEST");
        telemetry.addLine("Place robot at configured start pose before PLAY.");
        telemetry.addLine("A: clear tracks  B: collect UP TO 4  X: CANCEL");
        telemetry.addLine("Check clear path; automatic pickup has no obstacle avoidance.");
        telemetry.update();
    }

    @Override
    public void start() {
        visionSys.setMode(VisionSys.VisionMode.GAME_PIECES);
    }

    @Override
    public void loop() {
        boolean pressedA = gamepad1.a && !previousA;
        boolean pressedB = gamepad1.b && !previousB;
        boolean pressedX = gamepad1.x && !previousX;
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;

        boolean cancelledThisLoop = false;

        // Cancellation takes priority if B and X are pressed together.
        if (pressedX && pickupScheduled) {
            cancelPickup();
            cancelledThisLoop = true;
        } else if (pressedB && !pickupScheduled) {
            // Stop manual commands immediately, before Ivy starts the path.
            follower.stop();

            pickupCommand = sequential(
                    intake.startIntakeCommand(),
                    collection.collectUpTo(4),
                    intake.stopIntakeCommand()
            );
            pickupScheduled = true;
            Scheduler.schedule(pickupCommand);
            testMessage = "Pickup scheduled";
        }

        // MANUAL / AUTO EXCLUSION: only call manual() when Ivy does not own Pedro.
        if (!pickupScheduled && !cancelledThisLoop) {
            double forward = -gamepad1.left_stick_y * MANUAL_DRIVE_SCALE;
            double strafe = -gamepad1.left_stick_x * MANUAL_DRIVE_SCALE;
            double rotate = -gamepad1.right_stick_x * MANUAL_DRIVE_SCALE;
            follower.manual(forward, strafe, rotate);
        }

        // The follower must update during BOTH manual driving and path following.
        follower.update();
        visionSys.periodic();

        Pose robotPose = follower.pose();
        List<VisionSys.GamePieceObservation> observations =
                visionSys.getGamePiecePositions(robotPose);
        long nowMs = System.currentTimeMillis();

        // GamePieceCollection updates the SAME tracker while the command runs.
        // Avoid a duplicate update of the same observations in this OpMode.
        if (!pickupScheduled) {
            tracker.update(observations, nowMs);
            if (pressedA) {
                tracker.clear();
                testMessage = "Tracking memory cleared";
            }
        }

        // Starts/advances/completes Ivy commands once per OpMode loop.
        Scheduler.execute();

        // Give manual mode back only after the whole intake/collection sequence ends.
        if (pickupScheduled && pickupCommand != null
                && !Scheduler.isScheduled(pickupCommand)) {
            pickupScheduled = false;
            pickupCommand = null;
            testMessage = "Collection completed: " + collection.status();
        }

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Drive ownership", pickupScheduled ? "AUTO (B)" : "MANUAL");
        telemetry.addData("Message", testMessage);
        telemetry.addLine("A: clear tracks   B: collect UP TO 4   X: CANCEL");
        telemetry.addData("Manual stick scale", "%.2f", MANUAL_DRIVE_SCALE);

        telemetry.addLine("--- ROBOT ---");
        telemetry.addData("Robot X", "%.2f", robotPose.x());
        telemetry.addData("Robot Y", "%.2f", robotPose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(robotPose.heading()));

        telemetry.addLine("--- LIMELIGHT ---");
        telemetry.addData("Vision mode", visionSys.getCurrentMode());
        telemetry.addData("Vision status", visionSys.getGamePieceStatus());
        telemetry.addData("Raw Color Results", visionSys.getRawColorResults().size());
        telemetry.addData("Visible observations", observations.size());

        telemetry.addLine("--- GAME PIECE TRACKER ---");
        telemetry.addData("Total remembered", tracker.all().size());
        telemetry.addData("Available (recent)", tracker.available(nowMs).size());

        int shown = 0;
        for (GamePieceTracker.TrackedPiece ball : tracker.all()) {
            if (shown >= 10) break;
            telemetry.addData("Ball " + ball.id + " Field",
                    "X: %.2f  Y: %.2f", ball.x, ball.y);
            telemetry.addData("Ball " + ball.id + " Tracking",
                    "Visible: %s  Age: %d ms  Seen: %d",
                    ball.visibleThisUpdate, ball.ageMs(nowMs), ball.observationCount);
            shown++;
        }
        if (tracker.all().size() > shown) {
            telemetry.addData("Additional tracks hidden", tracker.all().size() - shown);
        }

        telemetry.addLine("--- COLLECTION PLANNER ---");
        telemetry.addData("Allowed ball X", "%.1f to %.1f", MIN_X, MAX_X);
        telemetry.addData("Allowed ball Y", "%.1f to %.1f", MIN_Y, MAX_Y);
        telemetry.addData("Pickup offset", "%.2f in", PICKUP_FORWARD_IN);

        int inQuadrant = 0;
        int validPickupPoses = 0;
        for (GamePieceTracker.TrackedPiece ball : tracker.available(nowMs)) {
            if (!planner.withinBounds(ball.x, ball.y)) continue;
            inQuadrant++;
            Pose goal = planner.pickupPose(robotPose, ball);
            if (planner.withinBounds(goal.x(), goal.y())) validPickupPoses++;
        }
        telemetry.addData("Fresh balls in quadrant", inQuadrant);
        telemetry.addData("In-bounds pickup poses", validPickupPoses);

        GamePieceTracker.TrackedPiece selected =
                planner.nearestTarget(tracker, robotPose, nowMs);
        if (selected == null) {
            telemetry.addData("Planner suggestion", "None");
        } else {
            Pose goal = planner.pickupPose(robotPose, selected);
            telemetry.addData("Planner suggestion", "Ball %d", selected.id);
            telemetry.addData("Target ball X/Y", "%.2f, %.2f", selected.x, selected.y);
            telemetry.addData("Pickup robot X/Y", "%.2f, %.2f", goal.x(), goal.y());
            telemetry.addData("Pickup heading", "%.1f deg", Math.toDegrees(goal.heading()));
            telemetry.addData("Distance to goal", "%.2f in",
                    Math.hypot(goal.x() - robotPose.x(), goal.y() - robotPose.y()));
        }

        telemetry.addLine("--- COLLECTION COMMAND ---");
        telemetry.addData("Status", collection.status());
        telemetry.addData("Planned IDs (fixed order)", collection.plannedTargetIds());
        telemetry.addData("Maximum requested balls", 4);
        telemetry.addData("Planned ball count", collection.plannedTargetCount());
        telemetry.addData("Current path leg", collection.currentLegNumber());
        telemetry.addData("Completed path legs", "%d / %d",
                collection.completedLegCount(), collection.plannedTargetCount());
        telemetry.addData("Presumed collected", collection.collectedCount());
        telemetry.addData("Last leg outcome", collection.lastLegResult());
        telemetry.addData("Active target ID", collection.activeTargetId());
        telemetry.addData("Assumed collected", collection.assumedCollected());
        telemetry.addData("Scan window", "%d ms after settle",
                GamePieceCollection.SCAN_WINDOW_MS);
        telemetry.addData("Path speed cap", "%.1f in/s",
                GamePieceCollection.MAX_TEST_VELOCITY_IN_PER_SEC);
        telemetry.addData("Pedro mode", follower.mode());
        Pose goal = collection.goalPose();
        if (goal != null) {
            telemetry.addData("Commanded robot goal", "X: %.2f Y: %.2f H: %.1f",
                    goal.x(), goal.y(), Math.toDegrees(goal.heading()));
            telemetry.addData("Current endpoint error", "%.2f in",
                    collection.endpointErrorIn());
        }
        if (Double.isFinite(collection.lastEndpointErrorIn())) {
            telemetry.addData("Final endpoint error", "%.2f in",
                    collection.lastEndpointErrorIn());
        }
        if (Double.isFinite(collection.lastIntakeErrorIn())) {
            telemetry.addData("Final intake-to-ball error", "%.2f in",
                    collection.lastIntakeErrorIn());
        }
        telemetry.update();
    }

    private void cancelPickup() {
        if (pickupCommand != null) {
            Scheduler.cancel(pickupCommand);
        }
        follower.stop();
        // Even if the Ivy sequence was interrupted before its normal end,
        // explicitly issue the intake stop command.
        Scheduler.schedule(intake.stopIntakeCommand());
        pickupCommand = null;
        pickupScheduled = false;
        testMessage = "Operator cancelled pickup";
    }

    @Override
    public void stop() {
        if (pickupScheduled && pickupCommand != null) {
            Scheduler.cancel(pickupCommand);
        }
        follower.stop();
        // Best effort: run the existing instantaneous stop-intake command once.
        Scheduler.schedule(intake.stopIntakeCommand());
        Scheduler.execute();
        Scheduler.reset();
        visionSys.stop();
    }
}
