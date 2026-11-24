package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class VisionSys {
    // ========================================================================
    // SHARED CAMERA STATE
    // ========================================================================

    private Limelight3A limelight;
    private LLResult latestResult;

    private Servo leftLimelightServo;
    private Servo rightLimelightServo;

    public enum VisionMode {
        APRIL_TAGS,
        GAME_PIECES
    }

    private VisionMode currentMode;

    // ========================================================================
    // APRILTAG CONFIGURATION
    // ========================================================================

    private static final int APRIL_TAG_PIPELINE = 0;
    private static final double APRIL_SERVO_1 = 0.0;
    private static final double APRIL_SERVO_2 = 1.0;

    // ========================================================================
    // GAME-PIECE CONFIGURATION
    // ========================================================================

    private static final int GAME_PIECE_PIPELINE = 1;
    private static final double GAME_SERVO_1 = 1.0;
    private static final double GAME_SERVO_2 = 0.0;

    public static double CAMERA_HEIGHT_IN = 10.625;
    public static double CAMERA_PITCH_DOWN_DEG = 28.0;

    // Lens position relative to robot center. Forward and left are positive.
    public static double CAMERA_FORWARD_IN = 6.325;
    public static double CAMERA_LEFT_IN = 0.0;

    // Ball center height above floor.
    public static double BALL_RADIUS_IN = 1.4;

    // Adjust only if physical testing reveals reversed image axes.
    public static double TX_SIGN = -1.0;
    public static double TY_SIGN = -1.0;

    // Reject outdated or extremely distant measurements.
    public static long MAX_RESULT_AGE_MS = 250;
    public static double MAX_CAMERA_DISTANCE_IN = 144.0;

    // ========================================================================
    // SHARED CAMERA METHODS
    // ========================================================================

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        leftLimelightServo = hardwareMap.get(Servo.class, "leftLimelightServo");
        rightLimelightServo = hardwareMap.get(Servo.class, "rightLimelightServo");

        limelight.start();

        setMode(VisionMode.APRIL_TAGS);
    }

    // Switch between vision modes
    public void setMode(VisionMode mode) {
        if (mode == null || mode == currentMode) {
            return;
        }

        currentMode = mode;
        latestResult = null;

        if (mode == VisionMode.APRIL_TAGS) {
            limelight.pipelineSwitch(APRIL_TAG_PIPELINE);

            leftLimelightServo.setPosition(APRIL_SERVO_1);
            rightLimelightServo.setPosition(APRIL_SERVO_2);
        } else {
            limelight.pipelineSwitch(GAME_PIECE_PIPELINE);

            leftLimelightServo.setPosition(GAME_SERVO_1);
            rightLimelightServo.setPosition(GAME_SERVO_2);
        }
    }

    // Refresh Limelight results
    public void periodic() {
        if (limelight != null) {
            latestResult = limelight.getLatestResult();
        }
    }

    // Check for a valid camera result
    public boolean hasValidResult() {
        return latestResult != null && latestResult.isValid();
    }

    // Get horizontal offset in degrees
    public double getTx() {
        return hasValidResult() ? latestResult.getTx() : 0.0;
    }

    // Get vertical offset in degrees
    public double getTy() {
        return hasValidResult() ? latestResult.getTy() : 0.0;
    }

    // Get current vision mode
    public VisionMode getCurrentMode() {
        return currentMode;
    }

    // Stop Limelight
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // ========================================================================
    // APRILTAG METHODS
    // ========================================================================

    // Check if any AprilTag is visible
    public boolean seesAprilTag() {
        return !getVisibleTagIds().isEmpty();
    }

    // Check for a specific AprilTag
    public boolean seesTag(int targetId) {
        for (int id : getVisibleTagIds()) {
            if (id == targetId) {
                return true;
            }
        }

        return false;
    }

    // Get all visible AprilTag IDs
    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();

        if (currentMode != VisionMode.APRIL_TAGS || !hasValidResult()) {
            return ids;
        }

        List<LLResultTypes.FiducialResult> tags =
                latestResult.getFiducialResults();

        if (tags == null) {
            return ids;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            ids.add(tag.getFiducialId());
        }

        return ids;
    }

    // Get the first visible AprilTag ID
    public int getPrimaryTagId() {
        List<Integer> ids = getVisibleTagIds();

        return ids.isEmpty() ? -1 : ids.get(0);
    }

    public double getTagAimRotation(int targetId) {
        if (currentMode != VisionMode.APRIL_TAGS || !hasValidResult()) {
            return 0.0;
        }

        List<LLResultTypes.FiducialResult> tags =
                latestResult.getFiducialResults();

        if (tags == null) {
            return 0.0;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == targetId) {
                double tx = tag.getTargetXDegrees() - 5.0; // -6.0

                // Deadband
                if (Math.abs(tx) < 1.0) {
                    return 0.0;
                }

                // Proportional rotation
                double rotation = -0.04 * tx;

                // Limit rotation speed
                return Math.max(-0.5, Math.min(0.5, rotation));
            }
        }
        return 0.0;
    }

    public boolean isTagOnTarget(int targetId) {
        if (currentMode != VisionMode.APRIL_TAGS || !hasValidResult()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> tags =
                latestResult.getFiducialResults();

        if (tags == null) {
            return false;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == targetId) {

                double tx = tag.getTargetXDegrees() - 5.0;

                return Math.abs(tx) < 4.0;
            }
        }

        return false;
    }

    // public boolean seesTagCommand(int tag) {
    //     return instant(() -> seesTag(tag));
    // }
    public Command seesTagCommand(int goalTag) {
        return Command.build()
                .setStart(() -> {
                    // executed on start
                })
                .setExecute(() -> {
                    // executed on execute
                })
                .setDone(() -> {
                    return seesTag(goalTag);
                })
                .setEnd(endCondition -> {
                    // executed on end
                })
                .requiring();
    }

    // ========================================================================
    // GAME-PIECE METHODS
    // ========================================================================

    public boolean seesGamePiece() {
        if (!hasFreshGamePieceResult()) {
            return false;
        }

        List<LLResultTypes.ColorResult> objects =
                latestResult.getColorResults();

        return objects != null && !objects.isEmpty();
    }

    private boolean hasFreshGamePieceResult() {
        if (currentMode != VisionMode.GAME_PIECES
                || latestResult == null
                || !latestResult.isValid()) {
            return false;
        }

        if (latestResult.getPipelineIndex() != GAME_PIECE_PIPELINE) {
            return false;
        }

        long age = latestResult.getStaleness();

        return age >= 0 && age <= MAX_RESULT_AGE_MS;
    }

    // Useful for telemetry/debugging
    public String getGamePieceStatus() {
        if (currentMode != VisionMode.GAME_PIECES) {
            return "Not in game-piece mode";
        }

        if (latestResult == null) {
            return "Waiting for Limelight";
        }

        if (latestResult.getPipelineIndex() != GAME_PIECE_PIPELINE) {
            return "Waiting for pipeline 1";
        }

        if (latestResult.getStaleness() > MAX_RESULT_AGE_MS) {
            return "Stale frame";
        }

        if (!latestResult.isValid()) {
            return "No valid target";
        }

        return "OK";
    }

    // ========================================================================
    // MULTI-PIECE DIAGNOSTICS (READ-ONLY)
    // ========================================================================

    /**
     * Raw color targets from the exact LLResult cached by periodic().
     * Deliberately does NOT apply the fresh-result or geometric filters.
     * Compare this list with getGamePiecePositions() in the SAME OpMode loop.
     */
    public List<LLResultTypes.ColorResult> getRawColorResults() {
        if (latestResult == null) {
            return new ArrayList<>();
        }
        List<LLResultTypes.ColorResult> results = latestResult.getColorResults();
        return results == null ? new ArrayList<>() : new ArrayList<>(results);
    }

    public int getLatestPipelineIndex() {
        return latestResult == null ? -1 : latestResult.getPipelineIndex();
    }

    public long getLatestResultAgeMs() {
        return latestResult == null ? -1L : latestResult.getStaleness();
    }

    public boolean isLatestResultValid() {
        return latestResult != null && latestResult.isValid();
    }

    public static class GamePieceObservation {
        // Individual camera measurements
        public final double tx;
        public final double ty;
        public final double area;

        // Position relative to robot center, in inches
        public final double robotForward;
        public final double robotLeft;

        // Estimated field position, in inches
        public final double fieldX;
        public final double fieldY;

        public GamePieceObservation(
                double tx,
                double ty,
                double area,
                double robotForward,
                double robotLeft,
                double fieldX,
                double fieldY) {
            this.tx = tx;
            this.ty = ty;
            this.area = area;
            this.robotForward = robotForward;
            this.robotLeft = robotLeft;
            this.fieldX = fieldX;
            this.fieldY = fieldY;
        }
    }
    public List<GamePieceObservation> getGamePiecePositions(Pose robotPose) {
        List<GamePieceObservation> pieces = new ArrayList<>();

        if (robotPose == null || !hasFreshGamePieceResult()) {
            return pieces;
        }

        List<LLResultTypes.ColorResult> results =
                latestResult.getColorResults();

        if (results == null) {
            return pieces;
        }

        // Camera geometry
        double heightDifference =
                CAMERA_HEIGHT_IN - BALL_RADIUS_IN;

        double pitch = Math.toRadians(CAMERA_PITCH_DOWN_DEG);

        double sinPitch = Math.sin(pitch);
        double cosPitch = Math.cos(pitch);

        // Pedro heading is already in radians
        double heading = robotPose.heading();

        double cosHeading = Math.cos(heading);
        double sinHeading = Math.sin(heading);

        for (LLResultTypes.ColorResult target : results) {
            // Individual angular offsets from the image principal point
            double tx = target.getTargetXDegreesNoCrosshair();
            double ty = target.getTargetYDegreesNoCrosshair();

            double txRad = Math.toRadians(tx * TX_SIGN);
            double tyRad = Math.toRadians(ty * TY_SIGN);

            double tanX = Math.tan(txRad);
            double tanY = Math.tan(tyRad);

            // Downward component of the camera ray
            double downComponent =
                    sinPitch - tanY * cosPitch;

            // Reject rays that do not intersect the target-height plane
            // safely below the horizon.
            if (downComponent <= 0.05) {
                continue;
            }

            // Ray intersection with the horizontal ball-center plane
            double forwardFromCamera =
                    heightDifference
                            * (cosPitch + tanY * sinPitch)
                            / downComponent;

            double rightFromCamera =
                    heightDifference
                            * tanX
                            / downComponent;

            // Reject invalid or excessively distant measurements
            if (!Double.isFinite(forwardFromCamera)
                    || !Double.isFinite(rightFromCamera)
                    || forwardFromCamera < 0
                    || Math.hypot(forwardFromCamera, rightFromCamera)
                    > MAX_CAMERA_DISTANCE_IN) {
                continue;
            }

            // Convert camera-relative location to robot-relative location
            double robotForward =
                    CAMERA_FORWARD_IN + forwardFromCamera;

            double robotLeft =
                    CAMERA_LEFT_IN - rightFromCamera;

            // Rotate into Pedro's field axes and add robot field position
            double fieldX =
                    robotPose.x()
                            + robotForward * cosHeading
                            - robotLeft * sinHeading;

            double fieldY =
                    robotPose.y()
                            + robotForward * sinHeading
                            + robotLeft * cosHeading;

            pieces.add(new GamePieceObservation(
                    tx,
                    ty,
                    target.getTargetArea(),
                    robotForward,
                    robotLeft,
                    fieldX,
                    fieldY
            ));
        }

        return pieces;
    }
}
