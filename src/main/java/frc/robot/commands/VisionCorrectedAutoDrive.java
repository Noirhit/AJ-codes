package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.*;

public class VisionCorrectedAutoDrive extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final double speed;
  private final double targetHeadingDegrees;

  public VisionCorrectedAutoDrive(CANDriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem, double speed, double targetHeadingDegrees) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.speed = speed;
    this.targetHeadingDegrees = targetHeadingDegrees;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double rotation = 0;
    if (visionSubsystem.hasTarget()) {
      double currentHeading = visionSubsystem.getEstimatedPose().getRotation().getDegrees();
      double headingError = targetHeadingDegrees - currentHeading;
      rotation = -AIM_KP * headingError;
      rotation = Math.max(-0.3, Math.min(0.3, rotation));
    }
    driveSubsystem.driveArcade(speed, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
