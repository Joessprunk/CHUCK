package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Picks pickup poses inside the configured field quadrant.
 *
 * A fixed route is selected from the latest tracked ball positions. Up to four
 * stops are considered, and every route leg is a straight-line Pedro path.
 * There is no obstacle avoidance and no mid-route replanning in this version.
 */
public class CollectionPlanner {

    private final double minX;
    private final double maxX;
    private final double minY;
    private final double maxY;
    private final double pickupForwardIn;
    private final PoseFactory radians = PoseFactory.radians();

    // Bounded permutation search: at most 12 * 11 * 10 * 9 routes at depth 4.
    // If many detections are remembered, favor the nearer candidates first.
    private static final int MAX_ROUTE_CANDIDATES = 12;
    private static final double AGE_COST_IN_PER_MS = 0.002;

    public CollectionPlanner(double minX, double maxX, double minY, double maxY,
                             double pickupForwardIn) {
        if (minX >= maxX || minY >= maxY || pickupForwardIn <= 0) {
            throw new IllegalArgumentException("Invalid quadrant bounds or intake offset");
        }
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
        this.pickupForwardIn = pickupForwardIn;
    }

    public double pickupForwardIn() { return pickupForwardIn; }

    public boolean withinBounds(double x, double y) {
        return Double.isFinite(x) && Double.isFinite(y)
                && x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** Face a ball and stop the robot center this distance behind its center. */
    public Pose pickupPose(Pose robotPose, GamePieceTracker.TrackedPiece ball) {
        double dx = ball.x - robotPose.x();
        double dy = ball.y - robotPose.y();
        double heading = Math.atan2(dy, dx);
        double x = ball.x - pickupForwardIn * Math.cos(heading);
        double y = ball.y - pickupForwardIn * Math.sin(heading);
        return radians.of(x, y, heading);
    }

    /** Retain the existing nearest-ball API for older telemetry/tests. */
    public GamePieceTracker.TrackedPiece nearestTarget(GamePieceTracker tracker,
                                                       Pose robotPose, long nowMs) {
        GamePieceTracker.TrackedPiece best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (GamePieceTracker.TrackedPiece ball : tracker.available(nowMs)) {
            if (!withinBounds(ball.x, ball.y)) continue;
            Pose goal = pickupPose(robotPose, ball);
            if (!withinBounds(goal.x(), goal.y())) continue;
            double cost = distance(robotPose, goal)
                    + AGE_COST_IN_PER_MS * ball.ageMs(nowMs);
            if (cost < bestCost) {
                bestCost = cost;
                best = ball;
            }
        }
        return best;
    }

    /** Frozen pickup coordinates: ball ID and robot destination at planning time. */
    public static class PickupStop {
        public final int ballId;
        public final double plannedBallX;
        public final double plannedBallY;
        public final Pose robotGoal;

        public PickupStop(GamePieceTracker.TrackedPiece ball, Pose robotGoal) {
            this.ballId = ball.id;
            this.plannedBallX = ball.x;
            this.plannedBallY = ball.y;
            this.robotGoal = robotGoal;
        }
    }

    /**
     * Existing strict-count route API, extended to 1..4 balls.
     * Returns an empty route if exactly count stops cannot be reached safely.
     */
    public List<PickupStop> planFixedRoute(GamePieceTracker tracker, Pose start,
                                           long nowMs, int count, double maxLegIn) {
        validateCount(count);
        return searchRoute(tracker, start, nowMs, count, count, maxLegIn);
    }

    /**
     * New variable-count API: find the longest feasible route with 1..maxCount
     * stops, breaking ties by total distance plus a small detection-age cost.
     * This prioritizes collecting more balls instead of merely picking the
     * shortest one-ball route when several reachable balls exist.
     */
    public List<PickupStop> planUpTo(GamePieceTracker tracker, Pose start,
                                     long nowMs, int maxCount, double maxLegIn) {
        validateCount(maxCount);
        return searchRoute(tracker, start, nowMs, 1, maxCount, maxLegIn);
    }

    private static void validateCount(int count) {
        if (count < 1 || count > 4) {
            throw new IllegalArgumentException("Pickup count must be between 1 and 4");
        }
    }

    private List<PickupStop> searchRoute(GamePieceTracker tracker, Pose start,
                                         long nowMs, int minCount, int maxCount,
                                         double maxLegIn) {
        if (tracker == null || start == null || !Double.isFinite(maxLegIn)
                || maxLegIn <= 0) {
            return Collections.emptyList();
        }

        List<GamePieceTracker.TrackedPiece> candidates = new ArrayList<>();
        for (GamePieceTracker.TrackedPiece ball : tracker.available(nowMs)) {
            if (withinBounds(ball.x, ball.y)) candidates.add(ball);
        }
        candidates.sort(Comparator.comparingDouble(ball ->
                Math.hypot(ball.x - start.x(), ball.y - start.y())));
        if (candidates.size() > MAX_ROUTE_CANDIDATES) {
            candidates = new ArrayList<>(candidates.subList(0, MAX_ROUTE_CANDIDATES));
        }
        if (candidates.isEmpty()) return Collections.emptyList();

        RouteSearch search = new RouteSearch(minCount, maxCount);
        search.expand(candidates, new boolean[candidates.size()],
                start, nowMs, maxLegIn, new ArrayList<PickupStop>(), 0.0);
        return Collections.unmodifiableList(search.bestRoute);
    }

    private final class RouteSearch {
        private final int minCount;
        private final int maxCount;
        private int bestCount = 0;
        private double bestCost = Double.POSITIVE_INFINITY;
        private List<PickupStop> bestRoute = Collections.emptyList();

        private RouteSearch(int minCount, int maxCount) {
            this.minCount = minCount;
            this.maxCount = maxCount;
        }

        private void expand(List<GamePieceTracker.TrackedPiece> candidates,
                            boolean[] used, Pose from, long nowMs,
                            double maxLegIn, List<PickupStop> route,
                            double cost) {
            int size = route.size();
            if (size >= minCount && (size > bestCount
                    || (size == bestCount && cost < bestCost))) {
                bestCount = size;
                bestCost = cost;
                bestRoute = new ArrayList<>(route);
            }
            if (size == maxCount) return;
            for (int i = 0; i < candidates.size(); i++) {
                if (used[i]) continue;
                GamePieceTracker.TrackedPiece ball = candidates.get(i);
                Pose goal = pickupPose(from, ball);
                if (!withinBounds(goal.x(), goal.y())) continue;

                double legIn = distance(from, goal);
                if (!Double.isFinite(legIn) || legIn > maxLegIn) continue;

                used[i] = true;
                route.add(new PickupStop(ball, goal));
                expand(candidates, used, goal, nowMs, maxLegIn, route,
                        cost + legIn + AGE_COST_IN_PER_MS * ball.ageMs(nowMs));
                route.remove(route.size() - 1);
                used[i] = false;
            }
        }
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }
}
