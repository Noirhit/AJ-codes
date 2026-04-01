package frc.robot.commands;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.*;

public class AlignToProcessor extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private int selectedTagId = -1;

  public AlignToProcessor(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    selectedTagId = -1;
    Set<Integer> visible = visionSubsystem.getVisibleTagIds();
    double bestDistance = Double.MAX_VALUE;
    for (int tagId : HUB_TAG_IDS) {
      if (visible.contains(tagId)) {
        Optional<Double> dist = visionSubsystem.getDistanceToTag(tagId);
        if (dist.isPresent() && dist.get() < bestDistance) {
          bestDistance = dist.get();
          selectedTagId = tagId;
        }
      }
    }
  }

  @Override
  public void execute() {
    if (selectedTagId == -1) {
      return;
    }
    Optional<Double> distanceOpt = visionSubsystem.getDistanceToTag(selectedTagId);
    if (distanceOpt.isEmpty()) {
      return;
    }
    double distanceError = distanceOpt.get() - HUB_SCORING_DISTANCE;
    double speed = Math.max(-0.4, Math.min(0.4, RANGE_KP * distanceError));
    double rotation = 0;
    Optional<Double> yawOpt = visionSubsystem.getTargetYaw(selectedTagId);
    if (yawOpt.isPresent()) {
      rotation = -AIM_KP * yawOpt.get();
      rotation = Math.max(-0.4, Math.min(0.4, rotation));
    }
    driveSubsystem.driveArcade(speed, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    if (selectedTagId == -1) {
      return true;
    }
    Optional<Double> distanceOpt = visionSubsystem.getDistanceToTag(selectedTagId);
    if (distanceOpt.isEmpty()) {
      return false;
    }
    double distanceError = Math.abs(distanceOpt.get() - HUB_SCORING_DISTANCE);
    Optional<Double> yawOpt = visionSubsystem.getTargetYaw(selectedTagId);
    boolean yawAligned = yawOpt.map(yaw -> Math.abs(yaw) < AIM_TOLERANCE_DEGREES).orElse(false);
    return distanceError < RANGE_TOLERANCE_METERS && yawAligned;
  }
}
