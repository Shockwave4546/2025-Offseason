package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera0 = new PhotonCamera("cam0");
    private final PhotonCamera camera1 = new PhotonCamera("cam1");
    private final PhotonCamera camera2 = new PhotonCamera("cam2");
    private final PhotonCamera camera3 = new PhotonCamera("cam3");

    // not final because we might set them null if the layout failed to load
    private PhotonPoseEstimator poseEstimator0;
    private PhotonPoseEstimator poseEstimator1;
    private PhotonPoseEstimator poseEstimator2;
    private PhotonPoseEstimator poseEstimator3;

    // Example transforms lets move these to constants later

    private static final double MAX_AMBIGUITY = 0.2;

    private final Transform3d robotToCamera0 = new Transform3d(
        new Translation3d(0.20, 0.0, 0.50),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );
    private final Transform3d robotToCamera1 = new Transform3d(
        new Translation3d(-0.20, 0.0, 0.50),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );
    private final Transform3d robotToCamera2 = new Transform3d(
        new Translation3d(0.0, 0.20, 0.50),
        new Rotation3d(0, 0, Math.toRadians(90))
    );
    private final Transform3d robotToCamera3 = new Transform3d(
        new Translation3d(0.0, -0.20, 0.50),
        new Rotation3d(0, 0, Math.toRadians(-90))
    );

    public VisionSubsystem() {
        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout for 2025 Reefscape", e.getStackTrace());
        }

        if (layout == null) {
            // if we couldn't load the layout, leave pose estimators null (caller must handle empty list)
            poseEstimator0 = poseEstimator1 = poseEstimator2 = poseEstimator3 = null;
            return;
        }

        // NOTE: constructor signature in your PhotonPoseEstimator: (layout, strategy, robotToCam)
        poseEstimator0 = new PhotonPoseEstimator(layout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera0);
        poseEstimator0.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        poseEstimator1 = new PhotonPoseEstimator(layout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera1);
        poseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        poseEstimator2 = new PhotonPoseEstimator(layout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera2);
        poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        poseEstimator3 = new PhotonPoseEstimator(layout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera3);
        poseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Get all available vision pose estimates from every camera.
     * Returns 0..4 estimates depending on what each camera sees.
     */
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        List<EstimatedRobotPose> poses = new ArrayList<>();

        if (poseEstimator0 != null) {
            poseEstimator0.update(camera0.getLatestResult()).ifPresent(poses::add);
        }
        if (poseEstimator1 != null) {
            poseEstimator1.update(camera1.getLatestResult()).ifPresent(poses::add);
        }
        if (poseEstimator2 != null) {
            poseEstimator2.update(camera2.getLatestResult()).ifPresent(poses::add);
        }
        if (poseEstimator3 != null) {
            poseEstimator3.update(camera3.getLatestResult()).ifPresent(poses::add);
        }

        return poses;
    }

    /**
     * Get best pose estimate from all available estimates.
     */

    public Optional<EstimatedRobotPose> getBestEstimatedPose() {
        List<EstimatedRobotPose> poses = getEstimatedGlobalPoses();
        return poses.stream()
        .min((a, b) -> {
            double aAmbiguity = a.targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average().orElse(1.0);
            double bAmbiguity = b.targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average().orElse(1.0);
            return Double.compare(aAmbiguity, bAmbiguity);
        });
    }

    /**
     * Get the best pose estimate if its ambiguity is below the MAX_AMBIGUITY threshold.
     * Otherwise, return empty.
     */
    public Optional<EstimatedRobotPose> getTrustedPose() {
        Optional<EstimatedRobotPose> best = getBestEstimatedPose();
    
        if (best.isPresent()) {
            double avgAmbiguity = best.get().targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average().orElse(1.0);
    
            if (avgAmbiguity <= MAX_AMBIGUITY) {
                return best;
            }
        }
        return Optional.empty();
    }
    

}
