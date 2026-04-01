package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterLaunchSequence extends SequentialCommandGroup {
  public ShooterLaunchSequence(ShooterSubsystem shooterSubsystem, CANFuelSubsystem fuelSubsystem) {
    addCommands(
        new ShooterSpinUp(shooterSubsystem).withTimeout(3),
        new ShooterLaunch(shooterSubsystem, fuelSubsystem));
  }
}
