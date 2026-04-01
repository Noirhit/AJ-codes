package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionLaunchSequence extends SequentialCommandGroup {
  public VisionLaunchSequence(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      ShooterSubsystem shooterSubsystem, CANFuelSubsystem fuelSubsystem) {
    addCommands(
        new ParallelCommandGroup(
            new AimAtTarget(driveSubsystem, visionSubsystem).withTimeout(3),
            new ShooterSpinUp(shooterSubsystem).withTimeout(3)),
        new ShooterLaunch(shooterSubsystem, fuelSubsystem));
  }
}
