package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.*;

public class DriveToAprilTag extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final int targetTagId;
  private final double targetDistanceMeters;

  public DriveToAprilTag(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      int targetTagId, double targetDistanceMeters) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.targetTagId = targetTagId;
    this.targetDistanceMeters = targetDistanceMeters;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    Optional<Double> distanceOpt = visionSubsystem.getDistanceToTag(targetTagId);
    if (distanceOpt.isEmpty()) {
      driveSubsystem.driveArcade(0, 0);
      return;
    }

    double distance = distanceOpt.get();
    double distanceError = distance - targetDistanceMeters;
    double speed = Math.max(-0.5, Math.min(0.5, RANGE_KP * distanceError));

    double rotation = 0;
    Optional<Double> yawOpt = visionSubsystem.getTargetYaw(targetTagId);
    if (yawOpt.isPresent()) {
      rotation = -AIM_KP * yawOpt.get();
      rotation = Math.max(-0.5, Math.min(0.5, rotation));
    }

    driveSubsystem.driveArcade(speed, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    Optional<Double> distanceOpt = visionSubsystem.getDistanceToTag(targetTagId);
    if (distanceOpt.isEmpty()) {
      return false;
    }
    double distanceError = Math.abs(distanceOpt.get() - targetDistanceMeters);
    Optional<Double> yawOpt = visionSubsystem.getTargetYaw(targetTagId);
    boolean yawAligned = yawOpt.map(yaw -> Math.abs(yaw) < AIM_TOLERANCE_DEGREES).orElse(false);
    return distanceError < RANGE_TOLERANCE_METERS && yawAligned;
  }
}
