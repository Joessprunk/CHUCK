package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Fixed-route collection: exact-count collectOne()/collectTwo(), or collectUpTo(1..4).
 *
 * The calling OpMode starts the intake ONCE before this command and stops it
 * ONCE afterward. The follower drives one to four separate straight-line segments.
 * Targets/goals are frozen when the route is first chosen; this version does
 * NOT replan when a ball moves. Without an intake sensor, pickup is assumed
 * from robot-center endpoint proximity, not physically confirmed.
 */
public class GamePieceCollection {

    public static long SERVO_SETTLE_MS = 800;
    public static long ACQUIRE_TIMEOUT_MS = 3000;
    public static long SCAN_WINDOW_MS = 700; // after camera servo settling
    public static long DRIVE_TIMEOUT_MS = 14000; // per path leg
    public static long BETWEEN_PICKUPS_MS = 250;
    public static double PICKUP_TOLERANCE_IN = 2.5; // commanded endpoint tolerance for presumed pickup

    public static double MAX_TEST_PATH_LENGTH_IN = 40.0; // per leg
    public static double MAX_TEST_VELOCITY_IN_PER_SEC = 16.0;
    public static double ENDPOINT_CORRECTION_TIMEOUT_MS = 800.0;

    private final VisionSys vision;
    private final GamePieceTracker tracker;
    private final CollectionPlanner planner;
    private final Follower follower;

    private long startMs;
    private long driveStartedMs;
    private long nextLegAfterMs;
    private int activeTargetId = -1;
    private int activeLegIndex;
    private int collectedCount;
    private int completedLegCount;
    private String lastLegResult = "No leg completed";
    private boolean legStarted;
    private boolean done;
    private boolean running;
    private boolean assumedCollected;
    private String status = "Not started";
    private List<CollectionPlanner.PickupStop> plannedStops = Collections.emptyList();
    private Pose commandedGoal;
    private double lastEndpointErrorIn = Double.NaN;
    private double lastIntakeErrorIn = Double.NaN;

    public GamePieceCollection(Follower follower, VisionSys vision,
                               GamePieceTracker tracker, CollectionPlanner planner) {
        this.follower = follower;
        this.vision = vision;
        this.tracker = tracker;
        this.planner = planner;
    }

    public String status() { return status; }
    public int activeTargetId() { return activeTargetId; }
    public boolean assumedCollected() { return assumedCollected; }
    public boolean isActive() { return running; }
    public int collectedCount() { return collectedCount; }
    public int completedLegCount() { return completedLegCount; }
    public String lastLegResult() { return lastLegResult; }
    public int plannedTargetCount() { return plannedStops.size(); }
    public int currentLegNumber() {
        return plannedStops.isEmpty() ? 0 : Math.min(activeLegIndex + 1, plannedStops.size());
    }
    public Pose goalPose() { return commandedGoal; }
    public double lastEndpointErrorIn() { return lastEndpointErrorIn; }
    public double lastIntakeErrorIn() { return lastIntakeErrorIn; }

    public String plannedTargetIds() {
        if (plannedStops.isEmpty()) return "None";
        StringBuilder result = new StringBuilder();
        for (CollectionPlanner.PickupStop stop : plannedStops) {
            if (result.length() > 0) result.append(" -> ");
            result.append(stop.ballId);
        }
        return result.toString();
    }

    public double endpointErrorIn() {
        if (commandedGoal == null) return Double.NaN;
        Pose robot = follower.pose();
        return Math.hypot(commandedGoal.x() - robot.x(),
                          commandedGoal.y() - robot.y());
    }

    /** Keep the original single-piece command available to existing OpModes. */
    public Command collectOne() { return buildCollection(1); }

    /** Keep the existing exact-two route command available. */
    public Command collectTwo() { return buildCollection(2, false); }

    /** Collect however many eligible balls are available, from 1 up to maxCount. */
    public Command collectUpTo(int maxCount) {
        if (maxCount < 1 || maxCount > 4) {
            throw new IllegalArgumentException("maxCount must be 1..4");
        }
        return buildCollection(maxCount, true);
    }

    private Command buildCollection(final int requestedCount) {
        return buildCollection(requestedCount, false);
    }

    private Command buildCollection(final int requestedCount, final boolean flexible) {
        return Command.build()
                .setStart(() -> {
                    startMs = System.currentTimeMillis();
                    driveStartedMs = 0;
                    nextLegAfterMs = 0;
                    activeTargetId = -1;
                    activeLegIndex = 0;
                    collectedCount = 0;
                    completedLegCount = 0;
                    lastLegResult = "No leg completed";
                    legStarted = false;
                    done = false;
                    running = true;
                    assumedCollected = false;
                    status = "Waiting for game-piece camera";
                    commandedGoal = null;
                    lastEndpointErrorIn = Double.NaN;
                    lastIntakeErrorIn = Double.NaN;
                    plannedStops = Collections.emptyList();

                    // Preserve existing tracked balls. A-button owns tracker resets.
                    // Last-seen ages still limit which tracks can be planned.
                    vision.setMode(VisionSys.VisionMode.GAME_PIECES);
                })
                .setExecute(() -> {
                    if (done) return;
                    long now = System.currentTimeMillis();
                    if (now - startMs < SERVO_SETTLE_MS) return;

                    // OpMode updates the follower and camera before the Ivy scheduler.
                    tracker.update(vision.getGamePiecePositions(follower.pose()), now);

                    if (plannedStops.isEmpty()) {
                        // Allow a short observation window so that a single
                        // camera frame does not prematurely lock a 1-ball route.
                        if (flexible && now - startMs < SERVO_SETTLE_MS + SCAN_WINDOW_MS) {
                            status = "Scanning for up to " + requestedCount + " balls";
                            return;
                        }

                        List<CollectionPlanner.PickupStop> route = flexible
                                ? planner.planUpTo(tracker, follower.pose(), now,
                                        requestedCount, MAX_TEST_PATH_LENGTH_IN)
                                : planner.planFixedRoute(tracker, follower.pose(), now,
                                        requestedCount, MAX_TEST_PATH_LENGTH_IN);
                        if (route.isEmpty() || (!flexible && route.size() != requestedCount)) {
                            status = flexible
                                    ? "Waiting for an eligible, reachable ball"
                                    : "Waiting for " + requestedCount + " eligible ball(s)";
                            if (now - startMs >= ACQUIRE_TIMEOUT_MS) {
                                status = flexible
                                        ? "No reachable eligible balls found"
                                        : "Not enough eligible balls for " + requestedCount + "-ball route";
                                done = true;
                            }
                            return;
                        }
                        plannedStops = route;
                        status = "Route locked (" + route.size() + " ball(s)): "
                                + plannedTargetIds();
                        startLeg(now);
                        return;
                    }

                    if (!legStarted) {
                        if (now >= nextLegAfterMs) startLeg(now);
                        return;
                    }

                    if (now - driveStartedMs > DRIVE_TIMEOUT_MS) {
                        lastEndpointErrorIn = endpointErrorIn();
                        status = String.format(Locale.US,
                                "Ball %d drive timeout (%.2f in endpoint error)",
                                activeTargetId, lastEndpointErrorIn);
                        done = true;
                        return;
                    }

                    // End-of-segment check. Keep running intake during the handoff.
                    if (!follower.following()) {
                        legStarted = false;
                        finishLeg(now);
                    }
                })
                .setDone(() -> done)
                .setEnd(endCondition -> {
                    if (!done) {
                        lastEndpointErrorIn = endpointErrorIn();
                        status = "Operator cancelled after " + collectedCount + " assumed pickup(s)";
                    }
                    follower.stop();
                    running = false;
                })
                .requiring();
    }

    private void startLeg(long now) {
        if (activeLegIndex >= plannedStops.size()) {
            done = true;
            status = "Route finished";
            return;
        }

        CollectionPlanner.PickupStop stop = plannedStops.get(activeLegIndex);
        Pose start = follower.pose();
        commandedGoal = stop.robotGoal;
        activeTargetId = stop.ballId;

        double legDistance = Math.hypot(
                commandedGoal.x() - start.x(), commandedGoal.y() - start.y());
        if (!Double.isFinite(legDistance) || legDistance > MAX_TEST_PATH_LENGTH_IN) {
            status = "Leg to ball " + activeTargetId + " exceeds test limit";
            done = true;
            return;
        }

        // If we're already essentially at the planned robot-center goal, avoid
        // submitting a near-zero-length path and just check pickup proximity.
        if (legDistance >= 0.25) {
            follower.follow(
                    Paths.line(start, commandedGoal)
                            .linear(start, commandedGoal)
                            .with(
                                    Constants.foresightConfig.maxVelocityConstraint.at(
                                            MAX_TEST_VELOCITY_IN_PER_SEC),
                                    Constants.foresightConfig.timeoutConstraint.at(
                                            ENDPOINT_CORRECTION_TIMEOUT_MS)
                            )
            );
        }
        driveStartedMs = now;
        legStarted = true;
        status = "Driving leg " + (activeLegIndex + 1) + "/" + plannedStops.size()
                + " to ball " + activeTargetId;
    }

    private void finishLeg(long now) {
        CollectionPlanner.PickupStop stop = plannedStops.get(activeLegIndex);
        lastEndpointErrorIn = endpointErrorIn();
        Pose robot = follower.pose();
        GamePieceTracker.TrackedPiece tracked = tracker.getById(stop.ballId);

        double intakeX = robot.x()
                + planner.pickupForwardIn() * Math.cos(robot.heading());
        double intakeY = robot.y()
                + planner.pickupForwardIn() * Math.sin(robot.heading());

        // Use the latest tracker location, if it still exists. No intake sensor:
        // reaching the expected location does not guarantee physical collection.
        double ballX = tracked != null ? tracked.x : stop.plannedBallX;
        double ballY = tracked != null ? tracked.y : stop.plannedBallY;
        lastIntakeErrorIn = Math.hypot(ballX - intakeX, ballY - intakeY);

        // The live track can move or disappear as the ball enters the intake.
        // An intake-to-track distance over tolerance is DIAGNOSTIC ONLY:
        // it must not end a two-ball route after leg 1.
        // Without an intake sensor, consider a pickup *presumed* successful
        // when Pedro reached the commanded robot-center pickup goal.
        boolean reachedCommandedGoal = Double.isFinite(lastEndpointErrorIn)
                && lastEndpointErrorIn <= PICKUP_TOLERANCE_IN;

        completedLegCount++;
        if (reachedCommandedGoal) {
            tracker.markCollected(stop.ballId);
            collectedCount++;
            assumedCollected = true;
        }

        lastLegResult = String.format(Locale.US,
                "Ball %d: %s; endpoint %.2f in, tracked-ball %.2f in",
                stop.ballId,
                reachedCommandedGoal ? "pickup assumed" : "pickup UNCONFIRMED",
                lastEndpointErrorIn, lastIntakeErrorIn);

        // The route contains 1..4 frozen stops. Complete a leg
        // even when its ball pickup is uncertain, so B is never required again
        // to start the second leg. X / path timeout / path-limit still stop it.
        activeLegIndex++;
        if (activeLegIndex >= plannedStops.size()) {
            status = String.format(Locale.US,
                    "Route complete: %d/%d legs, %d pickup(s) assumed",
                    completedLegCount, plannedStops.size(), collectedCount);
            done = true;
            return;
        }

        nextLegAfterMs = now + BETWEEN_PICKUPS_MS;
        status = "Completed leg " + completedLegCount
                + " (" + (reachedCommandedGoal ? "assumed collected" : "unconfirmed")
                + "); continuing to ball " + plannedStops.get(activeLegIndex).ballId;

    }
}
