package org.firstinspires.ftc.teamcode.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Maintains a persistent map of balls observed by VisionSys.
 *
 * First implementation: nearest-neighbor association, one match per camera
 * detection/track, and exponential position smoothing. No motion prediction yet.
 * Coordinates are Pedro field inches.
 */
public class GamePieceTracker {

    // Tune after field testing. Too small duplicates tracks; too large merges balls.
    public static double MATCH_RADIUS_IN = 4.0;
    public static double POSITION_ALPHA = 0.45;

    // Unseen balls remain candidates for this long. Balls may roll, so old
    // positions become less trustworthy over time.
    public static long USABLE_AGE_MS = 8000;
    public static long MEMORY_AGE_MS = 20000;

    public static class TrackedPiece {
        public final int id;
        public double x;
        public double y;
        public final long firstSeenMs;
        public long lastSeenMs;
        public int observationCount;
        public boolean collected;
        public boolean visibleThisUpdate;

        private TrackedPiece(int id, double x, double y, long nowMs) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.firstSeenMs = nowMs;
            this.lastSeenMs = nowMs;
            this.observationCount = 1;
            this.collected = false;
            this.visibleThisUpdate = true;
        }

        public long ageMs(long nowMs) {
            return Math.max(0, nowMs - lastSeenMs);
        }
    }

    private final List<TrackedPiece> tracks = new ArrayList<>();
    private int nextId = 1;

    public void clear() {
        tracks.clear();
        nextId = 1;
    }

    /** Call once per OpMode loop, after visionSys.periodic(). */
    public void update(List<VisionSys.GamePieceObservation> detections) {
        update(detections, System.currentTimeMillis());
    }

    public void update(List<VisionSys.GamePieceObservation> detections, long nowMs) {
        for (TrackedPiece piece : tracks) {
            piece.visibleThisUpdate = false;
        }

        // Use a track at most once per update, so two nearby balls do not
        // overwrite the same tracked ball in a single camera frame.
        Set<Integer> assignedTrackIds = new HashSet<>();
        if (detections != null) {
            for (VisionSys.GamePieceObservation observation : detections) {
                if (observation == null
                        || !Double.isFinite(observation.fieldX)
                        || !Double.isFinite(observation.fieldY)) {
                    continue;
                }

                TrackedPiece closest = null;
                double bestDistance = MATCH_RADIUS_IN;

                for (TrackedPiece existing : tracks) {
                    if (assignedTrackIds.contains(existing.id)) {
                        continue;
                    }
                    double distance = Math.hypot(
                            existing.x - observation.fieldX,
                            existing.y - observation.fieldY);
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        closest = existing;
                    }
                }

                if (closest == null) {
                    TrackedPiece created = new TrackedPiece(
                            nextId++, observation.fieldX, observation.fieldY, nowMs);
                    tracks.add(created);
                    assignedTrackIds.add(created.id);
                } else {
                    assignedTrackIds.add(closest.id);
                    closest.lastSeenMs = nowMs;
                    closest.visibleThisUpdate = true;
                    closest.observationCount++;

                    // Don't reactivate an assumed-collected ball that is still
                    // visible just after the intake passes it.
                    if (!closest.collected) {
                        closest.x += POSITION_ALPHA * (observation.fieldX - closest.x);
                        closest.y += POSITION_ALPHA * (observation.fieldY - closest.y);
                    }
                }
            }
        }

        // Prevent the list growing for an entire match. Retention is deliberately
        // longer than candidate age so temporarily lost targets keep their IDs.
        tracks.removeIf(p -> nowMs - p.lastSeenMs > MEMORY_AGE_MS);
    }

    public List<TrackedPiece> all() {
        return Collections.unmodifiableList(tracks);
    }

    public List<TrackedPiece> available(long nowMs) {
        List<TrackedPiece> result = new ArrayList<>();
        for (TrackedPiece piece : tracks) {
            if (!piece.collected && piece.ageMs(nowMs) <= USABLE_AGE_MS) {
                result.add(piece);
            }
        }
        return result;
    }

    public TrackedPiece getById(int id) {
        for (TrackedPiece p : tracks) {
            if (p.id == id) {
                return p;
            }
        }
        return null;
    }

    /** Assumes pickup; replace this with a sensor confirmation later. */
    public void markCollected(int id) {
        TrackedPiece piece = getById(id);
        if (piece != null) {
            piece.collected = true;
        }
    }
}
