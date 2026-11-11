package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.*;

public class AimAtTarget extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final int targetTagId; // -1 = nearest hub tag, anything else = specific tag

  // ── Constructor for TELEOP: aims at nearest hub tag ──
  public AimAtTarget(CANDriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem) {
    this(driveSubsystem, visionSubsystem, -1);
  }

  // ── Constructor for AUTO: aims at a specific tag ID ──
  public AimAtTarget(CANDriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem, int targetTagId) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.targetTagId = targetTagId;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double yaw;

    if (targetTagId == -1) {
      // Teleop mode — aim at nearest visible hub tag
      var target = visionSubsystem.getBestHubTarget();
      if (target.isEmpty()) {
        driveSubsystem.driveArcade(0, 0);
        return;
      }
      yaw = target.get().getYaw();
    } else {
      // Auto mode — aim at specific tag only
      var yawOpt = visionSubsystem.getTargetYaw(targetTagId);
      if (yawOpt.isEmpty()) {
        driveSubsystem.driveArcade(0, 0);
        return;
      }
      yaw = yawOpt.get();
    }

    double rotation = MathUtil.clamp(-AIM_KP * yaw, -0.5, 0.5);
    driveSubsystem.driveArcade(0, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    if (targetTagId == -1) {
      // Teleop: finish when nearest hub tag is centered
      return visionSubsystem.getBestHubTarget()
          .map(t -> Math.abs(t.getYaw()) < AIM_TOLERANCE_DEGREES)
          .orElse(false);
    } else {
      // Auto: finish only when the specific tag is centered
      return visionSubsystem.getTargetYaw(targetTagId)
          .map(yaw -> Math.abs(yaw) < AIM_TOLERANCE_DEGREES)
          .orElse(false);
    }
  }
}