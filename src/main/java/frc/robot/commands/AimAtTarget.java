package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.*;

public class AimAtTarget extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;

  public AimAtTarget(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    visionSubsystem.getBestTarget().ifPresent(target -> {
      double yaw = target.getYaw();
      double rotation = -AIM_KP * yaw;
      rotation = Math.max(-0.5, Math.min(0.5, rotation));
      driveSubsystem.driveArcade(0, rotation);
    });
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    return visionSubsystem.getBestTarget()
        .map(target -> Math.abs(target.getYaw()) < AIM_TOLERANCE_DEGREES)
        .orElse(false);
  }
}
