package frc.robot.subsystems;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera frontCamera;
  private final PhotonCamera rearCamera;
  private final PhotonPoseEstimator frontPoseEstimator;
  private final PhotonPoseEstimator rearPoseEstimator;
  private final AprilTagFieldLayout fieldLayout;

  private Optional<EstimatedRobotPose> latestFrontPose = Optional.empty();
  private Optional<EstimatedRobotPose> latestRearPose = Optional.empty();
  private Pose2d estimatedPose = new Pose2d();
  private boolean hasTarget = false;
  private PhotonTrackedTarget bestTarget = null;
  private final Set<Integer> visibleTagIds = new HashSet<>();

  public VisionSubsystem() {
    frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    frontPoseEstimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        FRONT_CAMERA_TO_ROBOT);
    frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    rearPoseEstimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        REAR_CAMERA_TO_ROBOT);
    rearPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    visibleTagIds.clear();
    hasTarget = false;
    bestTarget = null;

    latestFrontPose = processCameraResult(frontCamera, frontPoseEstimator, "Front");
    latestRearPose = processCameraResult(rearCamera, rearPoseEstimator, "Rear");

    if (latestFrontPose.isPresent()) {
      estimatedPose = latestFrontPose.get().estimatedPose.toPose2d();
    } else if (latestRearPose.isPresent()) {
      estimatedPose = latestRearPose.get().estimatedPose.toPose2d();
    }

    SmartDashboard.putBoolean("Vision Has Target", hasTarget);
    SmartDashboard.putNumber("Vision Pose X", estimatedPose.getX());
    SmartDashboard.putNumber("Vision Pose Y", estimatedPose.getY());
    SmartDashboard.putNumber("Vision Pose Rotation", estimatedPose.getRotation().getDegrees());
    SmartDashboard.putString("Visible Tags", visibleTagIds.toString());
  }

  private Optional<EstimatedRobotPose> processCameraResult(
      PhotonCamera camera, PhotonPoseEstimator poseEstimator, String label) {

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    if (!latestResult.hasTargets()) {
      return Optional.empty();
    }

    hasTarget = true;

    for (PhotonTrackedTarget target : latestResult.getTargets()) {
      int tagId = target.getFiducialId();
      visibleTagIds.add(tagId);

      if (bestTarget == null || target.getArea() > bestTarget.getArea()) {
        bestTarget = target;
      }

      SmartDashboard.putNumber(label + " Tag " + tagId + " Yaw", target.getYaw());
      SmartDashboard.putNumber(label + " Tag " + tagId + " Area", target.getArea());
    }

    Optional<EstimatedRobotPose> estimate = poseEstimator.update(latestResult);

    if (estimate.isPresent()) {
      Pose3d pose = estimate.get().estimatedPose;
      boolean isMultiTag = latestResult.getMultiTagResult().isPresent();

      if (!isMultiTag) {
        PhotonTrackedTarget primary = latestResult.getBestTarget();
        if (primary != null && primary.getPoseAmbiguity() > MAX_POSE_AMBIGUITY) {
          return Optional.empty();
        }
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(primary.getFiducialId());
        if (tagPose.isPresent()) {
          double distance = tagPose.get().toPose2d().getTranslation()
              .getDistance(pose.toPose2d().getTranslation());
          if (distance > MAX_POSE_DISTANCE_METERS) {
            return Optional.empty();
          }
        }
      }

      SmartDashboard.putNumber(label + " Pose X", pose.getX());
      SmartDashboard.putNumber(label + " Pose Y", pose.getY());
    }

    return estimate;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(bestTarget);
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Optional<EstimatedRobotPose> getFrontPose() {
    return latestFrontPose;
  }

  public Optional<EstimatedRobotPose> getRearPose() {
    return latestRearPose;
  }

  public Set<Integer> getVisibleTagIds() {
    return visibleTagIds;
  }

  public Optional<Double> getTargetYaw(int tagId) {
    for (PhotonPipelineResult result : frontCamera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == tagId) {
          return Optional.of(target.getYaw());
        }
      }
    }
    for (PhotonPipelineResult result : rearCamera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == tagId) {
          return Optional.of(target.getYaw());
        }
      }
    }
    return Optional.empty();
  }

  public Optional<Double> getDistanceToTag(int tagId) {
    Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
    if (tagPose.isEmpty() || !hasTarget) {
      return Optional.empty();
    }
    double distance = tagPose.get().toPose2d().getTranslation()
        .getDistance(estimatedPose.getTranslation());
    return Optional.of(distance);
  }

  public void resetPose(Pose2d pose) {
    this.estimatedPose = pose;
  }
}
