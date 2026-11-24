package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamePieceTracker;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;

import java.util.List;
import java.util.Locale;

/**
 * Diagnoses missing second ball WITHOUT operating the intake or autonomous follower.
 *
 * IMPORTANT: VisionSys.periodic() is called only once per loop. Raw color targets
 * and localized observations are both read from that SAME cached LLResult.
 *
 * A = clear tracker + sample statistics.
 * B and X do nothing in this diagnostic OpMode.
 */
@TeleOp(name = "Game Piece Detection DIAGNOSTIC", group = "Testing")
public class GamePieceDetectionDiagnostic extends OpMode {

    private final VisionSys visionSys = new VisionSys();
    private final GamePieceTracker tracker = new GamePieceTracker();
    private Follower follower;

    private static final double DRIVE_SCALE = 0.40;
    private static final double MIN_X = -3.0;
    private static final double MAX_X = 70.0;
    private static final double MIN_Y = -3.0;
    private static final double MAX_Y = 70.0;

    // Matches the starting pose from the earlier working tests. Move robot to
    // this actual field pose before init, or edit these 3 numbers for your setup.
    private final Pose startPose = PoseFactory.degrees().of(57.3029, 9.5635, 270);

    private boolean previousA;
    private int samples;
    private int samplesWithTwoRaw;
    private int samplesWithTwoLocalized;

    @Override
    public void init() {
        visionSys.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        tracker.clear();
        telemetry.addLine("DETECTION DIAGNOSTIC - no auto pickup, no intake");
        telemetry.addLine("A: reset counts/tracker; manual drive = 40%");
        telemetry.addLine("Place robot at configured starting pose.");
        telemetry.update();
    }

    @Override
    public void start() {
        visionSys.setMode(VisionSys.VisionMode.GAME_PIECES);
    }

    @Override
    public void loop() {
        // Same preferred stick signs as the successful pickup test.
        follower.manual(
                -gamepad1.left_stick_y * DRIVE_SCALE,
                -gamepad1.left_stick_x * DRIVE_SCALE,
                -gamepad1.right_stick_x * DRIVE_SCALE);
        follower.update();

        // ONLY ONE camera poll. All diagnostic reads below use its cached result.
        visionSys.periodic();
        Pose pose = follower.pose();

        List<LLResultTypes.ColorResult> raw = visionSys.getRawColorResults();
        List<VisionSys.GamePieceObservation> localized =
                visionSys.getGamePiecePositions(pose);

        long now = System.currentTimeMillis();
        tracker.update(localized, now);

        if (gamepad1.a && !previousA) {
            tracker.clear();
            samples = 0;
            samplesWithTwoRaw = 0;
            samplesWithTwoLocalized = 0;
        }
        previousA = gamepad1.a;

        samples++;
        if (raw.size() >= 2) samplesWithTwoRaw++;
        if (localized.size() >= 2) samplesWithTwoLocalized++;

        telemetry.addLine("--- SAME CAMERA RESULT ---");
        telemetry.addData("Status", visionSys.getGamePieceStatus());
        telemetry.addData("Pipeline", visionSys.getLatestPipelineIndex());
        telemetry.addData("Result valid", visionSys.isLatestResultValid());
        telemetry.addData("Result age (ms)", visionSys.getLatestResultAgeMs());
        telemetry.addData("RAW color results", raw.size());
        telemetry.addData("LOCALIZED observations", localized.size());

        telemetry.addLine("--- EACH RAW TARGET AND GEOMETRY RESULT ---");
        for (int i = 0; i < Math.min(raw.size(), 4); i++) {
            LLResultTypes.ColorResult target = raw.get(i);
            telemetry.addData("Raw target " + (i + 1),
                    "tx %.1f  ty %.1f  area %.2f  %s",
                    target.getTargetXDegreesNoCrosshair(),
                    target.getTargetYDegreesNoCrosshair(),
                    target.getTargetArea(),
                    geometryDiagnosis(target));
        }
        if (raw.size() > 4) {
            telemetry.addData("Unshown raw targets", raw.size() - 4);
        }

        telemetry.addLine("--- LOCALIZED / TRACKING ---");
        int localizedInQuadrant = 0;
        for (VisionSys.GamePieceObservation o : localized) {
            if (o.fieldX >= MIN_X && o.fieldX <= MAX_X
                    && o.fieldY >= MIN_Y && o.fieldY <= MAX_Y) {
                localizedInQuadrant++;
            }
        }
        telemetry.addData("Localized inside quadrant", localizedInQuadrant);
        telemetry.addData("Tracks remembered", tracker.all().size());
        telemetry.addData("Tracks recent/usable", tracker.available(now).size());
        telemetry.addData("Samples with >=2 RAW", "%d of %d", samplesWithTwoRaw, samples);
        telemetry.addData("Samples with >=2 LOCALIZED", "%d of %d",
                samplesWithTwoLocalized, samples);
        telemetry.addData("Robot field pose", "X %.1f  Y %.1f  H %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("A: reset counts; B/X: no action; NO automatic driving");
        telemetry.update();
    }

    /**
     * Reproduce the SAME geometric acceptance tests as VisionSys so that a
     * raw target can be assigned a concrete rejection reason. This method
     * must be kept in sync if VisionSys localization rules ever change.
     */
    private static String geometryDiagnosis(LLResultTypes.ColorResult target) {
        double tx = target.getTargetXDegreesNoCrosshair();
        double ty = target.getTargetYDegreesNoCrosshair();
        double txRad = Math.toRadians(tx * VisionSys.TX_SIGN);
        double tyRad = Math.toRadians(ty * VisionSys.TY_SIGN);
        double tanX = Math.tan(txRad);
        double tanY = Math.tan(tyRad);

        double pitch = Math.toRadians(VisionSys.CAMERA_PITCH_DOWN_DEG);
        double sinPitch = Math.sin(pitch);
        double cosPitch = Math.cos(pitch);
        double downComponent = sinPitch - tanY * cosPitch;

        // Same first rejection as VisionSys.getGamePiecePositions().
        if (downComponent <= 0.05) {
            return "REJECT: ray above horizon";
        }

        double heightDiff = VisionSys.CAMERA_HEIGHT_IN - VisionSys.BALL_RADIUS_IN;
        double forward = heightDiff * (cosPitch + tanY * sinPitch) / downComponent;
        double right = heightDiff * tanX / downComponent;
        double distance = Math.hypot(forward, right);

        // Same second rejection as VisionSys, with a more specific reason.
        if (!Double.isFinite(forward) || !Double.isFinite(right)) {
            return "REJECT: invalid projection";
        }
        if (forward < 0) {
            return "REJECT: behind camera";
        }
        if (distance > VisionSys.MAX_CAMERA_DISTANCE_IN) {
            return String.format(Locale.US, "REJECT: %.1f in > max range", distance);
        }
        return String.format(Locale.US, "PASS: %.1f in from lens", distance);
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
        visionSys.stop();
    }
}
