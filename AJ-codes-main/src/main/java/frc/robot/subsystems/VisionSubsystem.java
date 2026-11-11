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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
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
  private PhotonPipelineResult latestFrontResult = null;
  private PhotonPipelineResult latestRearResult = null;

  public VisionSubsystem() {
    frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

    // Publish camera streams to SmartDashboard
    // Add a "Camera Stream" widget in SmartDashboard and select the camera name
    CameraServer.startAutomaticCapture(
        new HttpCamera("Front AprilTag", FRONT_CAMERA_STREAM_URL));
    CameraServer.startAutomaticCapture(
        new HttpCamera("Rear AprilTag", REAR_CAMERA_STREAM_URL));
    CameraServer.startAutomaticCapture(
        new HttpCamera("Fuel Tray", FUEL_TRAY_STREAM_URL));

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

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

    SmartDashboard.putBoolean("Vision Sees Tag", hasTarget);
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
    // Cache the result so getTargetYaw() can use it without re-reading
    if (camera == frontCamera) latestFrontResult = latestResult;
    else latestRearResult = latestResult;
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

  public Optional<PhotonTrackedTarget> getBestHubTarget() {
    PhotonTrackedTarget best = null;
    for (int id : HUB_TAG_IDS) {
      if (latestFrontResult != null) {
        for (PhotonTrackedTarget t : latestFrontResult.getTargets()) {
          if (t.getFiducialId() == id) {
            if (best == null || t.getArea() > best.getArea()) best = t;
          }
        }
      }
      if (latestRearResult != null) {
        for (PhotonTrackedTarget t : latestRearResult.getTargets()) {
          if (t.getFiducialId() == id) {
            if (best == null || t.getArea() > best.getArea()) best = t;
          }
        }
      }
    }
    return Optional.ofNullable(best);
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

  public Optional<Double> getDistanceToNearestHubTag() {
    double closest = Double.MAX_VALUE;
    for (int id : HUB_TAG_IDS) {
      Optional<Double> dist = getDistanceToTag(id);
      if (dist.isPresent() && dist.get() < closest) {
        closest = dist.get();
      }
    }
    if (closest == Double.MAX_VALUE) return Optional.empty();
    SmartDashboard.putNumber("Hub Distance (m)", closest);
    return Optional.of(closest);
  }

  /** Find a target with the given tag ID, checking cached front result first, then rear. */
  private Optional<PhotonTrackedTarget> findTargetInAnyCamera(int tagId) {
    if (latestFrontResult != null) {
      for (PhotonTrackedTarget target : latestFrontResult.getTargets()) {
        if (target.getFiducialId() == tagId) return Optional.of(target);
      }
    }
    if (latestRearResult != null) {
      for (PhotonTrackedTarget target : latestRearResult.getTargets()) {
        if (target.getFiducialId() == tagId) return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  public Optional<Double> getDistanceToTag(int tagId) {
    return findTargetInAnyCamera(tagId)
        .map(t -> t.getBestCameraToTarget().getTranslation().getNorm());
  }

  public Optional<Double> getTargetYaw(int tagId) {
    return findTargetInAnyCamera(tagId).map(PhotonTrackedTarget::getYaw);
  }

  public void resetPose(Pose2d pose) {
    this.estimatedPose = pose;
  }
/**
 * Returns the correct Hub AprilTag ID to aim at based on current alliance.
 * Call this at the START of auto, not during — alliance data may not be
 * ready instantly on boot.
 */
  public static int getAutoTargetTagId() {
      var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
      if (alliance.isPresent()) {
          return alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red
             ? AUTO_TARGET_TAG_RED
              : AUTO_TARGET_TAG_BLUE;
     }
      // Default to red if alliance unknown — better than crashing
      return AUTO_TARGET_TAG_RED;
  }
}
