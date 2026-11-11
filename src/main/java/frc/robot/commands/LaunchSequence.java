package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LaunchSequence extends SequentialCommandGroup {

  // Teleop launch — reads launcher RPM from SmartDashboard, falls back to LAUNCHING_LAUNCHER_RPM
  public LaunchSequence(CANFuelSubsystem fuelSubsystem) {
    addCommands(
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem));
  }

  // Auto launch — fixed RPM passed in (e.g. AUTO_LAUNCHING_LAUNCHER_RPM)
  public LaunchSequence(CANFuelSubsystem fuelSubsystem, double fixedRpm) {
    addCommands(
        new SpinUp(fuelSubsystem, () -> fixedRpm).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem, () -> fixedRpm));
  }

  // Distance-based launch — measures hub distance via vision, looks up RPM in table.
  // Falls back to LAUNCHING_LAUNCHER_RPM if no hub tag is visible.
  // Reserved for future use — not currently active in auto or teleop.
  public LaunchSequence(CANFuelSubsystem fuelSubsystem, VisionSubsystem visionSubsystem) {
    double[] rpm = { FuelConstants.LAUNCHING_LAUNCHER_RPM };

    addCommands(
        new InstantCommand(() -> visionSubsystem.getDistanceToNearestHubTag()
            .ifPresent(dist -> rpm[0] = FuelConstants.LAUNCHER_RPM_TABLE.get(dist))),

        new SpinUp(fuelSubsystem, () -> rpm[0]).withTimeout(FuelConstants.SPIN_UP_SECONDS),

        new Launch(fuelSubsystem, () -> rpm[0]));
  }
}
