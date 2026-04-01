package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TwoPieceAuto extends SequentialCommandGroup {
  public TwoPieceAuto(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      ShooterSubsystem shooterSubsystem, CANFuelSubsystem fuelSubsystem) {
    addCommands(
        // First piece: aim and shoot from starting position
        new ParallelCommandGroup(
            new AimAtTarget(driveSubsystem, visionSubsystem).withTimeout(2),
            new ShooterSpinUp(shooterSubsystem).withTimeout(2)),
        new ShooterLaunch(shooterSubsystem, fuelSubsystem).withTimeout(2),

        // Drive forward to pick up second piece
        new AutoDrive(driveSubsystem, 0.4, 0).withTimeout(1.5),

        // Intake the second piece
        new Intake(fuelSubsystem).withTimeout(2),

        // Drive back and score again
        new AutoDrive(driveSubsystem, -0.4, 0).withTimeout(1.5),

        // Second piece: aim and shoot
        new ParallelCommandGroup(
            new AimAtTarget(driveSubsystem, visionSubsystem).withTimeout(2),
            new ShooterSpinUp(shooterSubsystem).withTimeout(2)),
        new ShooterLaunch(shooterSubsystem, fuelSubsystem).withTimeout(2));
  }
}
