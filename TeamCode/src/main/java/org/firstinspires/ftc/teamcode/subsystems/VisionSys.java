
package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class VisionSys {

    private Limelight3A limelight;
    private LLResult latestResult;

    private Servo leftLimelightServo;
    private Servo rightLimelightServo;

    public enum VisionMode {
        APRIL_TAGS,
        GAME_PIECES
    }

    private VisionMode currentMode;

    private static final int APRIL_TAG_PIPELINE = 0;
    private static final int GAME_PIECE_PIPELINE = 1;

    private static final double APRIL_SERVO_1 = 0.0;
    private static final double APRIL_SERVO_2 = 1.0;

    private static final double GAME_SERVO_1 = 1.0;
    private static final double GAME_SERVO_2 = 0.0;


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


    // Check if a game piece is visible
    public boolean seesGamePiece() {

        if (currentMode != VisionMode.GAME_PIECES || !hasValidResult()) {
            return false;
        }

        List<LLResultTypes.DetectorResult> objects =
                latestResult.getDetectorResults();

        return objects != null && !objects.isEmpty();
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

    // Aim between two specific AprilTags
    public double getTargetRotation(int tag1, int tag2) {

        if (currentMode != VisionMode.APRIL_TAGS || !hasValidResult()) {
            return 0.0;
        }

        List<LLResultTypes.FiducialResult> tags =
                latestResult.getFiducialResults();

        if (tags == null) {
            return 0.0;
        }

        double tx1 = Double.NaN;
        double tx2 = Double.NaN;

        // Find both target tags
        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag.getFiducialId() == tag1) {
                tx1 = tag.getTargetXDegrees();
            }

            if (tag.getFiducialId() == tag2) {
                tx2 = tag.getTargetXDegrees();
            }
        }

        // Require both tags to be visible
        if (Double.isNaN(tx1) || Double.isNaN(tx2)) {
            return 0.0;
        }

        // Calculate midpoint between the two tags
        double tx = (tx1 + tx2) / 2.0;

        // Deadband
        if (Math.abs(tx) < 1.0) {
            return 0.0;
        }

        // Proportional rotation
        double rotation = -0.025 * tx;

        // Limit rotation speed
        return Math.max(-0.5, Math.min(0.5, rotation));
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

                double tx = tag.getTargetXDegrees();

                // Deadband
                if (Math.abs(tx) < 1.0) {
                    return 0.0;
                }

                // Proportional rotation
                double rotation = -0.03 * tx;

                // Limit rotation speed
                return Math.max(-0.5, Math.min(0.5, rotation));
            }
        }
        return 0.0;
    }
//    public boolean seesTagCommand(int tag){
//        return instant(() -> seesTag(tag));
//    }


}

