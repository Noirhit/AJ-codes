// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleAuto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public ExampleAuto(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      CANFuelSubsystem fuelSubsystem) {
    addCommands(
    // Drive forward for .25 seconds to clear the starting position
    new AutoDrive(driveSubsystem, 0.5, 0.0).withTimeout(.25),
    // Rotate to face the nearest HUB AprilTag (spinners stay off until aligned)
    new AimAtTarget(driveSubsystem, visionSubsystem).withTimeout(2),
    // Spin up rollers then launch
    new LaunchSequence(fuelSubsystem).withTimeout(5));
  }
}
