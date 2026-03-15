// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  CommandXboxController controller;

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // 1. Deadband: ignores tiny joystick inputs so the robot doesn't drift at rest
  // 2. Scaling: multiplies by DRIVE_SCALING and ROTATION_SCALING to limit max speed
  // 3. Squared: makes small movements more precise while still allowing full speed at max push
  @Override
  public void execute() {
    double currentDriveScaling = controller.leftBumper().getAsBoolean() ? DRIVE_TURBO_SCALING : DRIVE_SCALING;
    double currentRotationScaling = controller.rightBumper().getAsBoolean() ? ROTATION_TURBO_SCALING : ROTATION_SCALING;

    // Apply deadband first to eliminate drift, then scale
    double speed = MathUtil.applyDeadband(-controller.getLeftY(), DEADBAND) * currentDriveScaling;
    double rotation = MathUtil.applyDeadband(-controller.getRightX(), DEADBAND) * currentRotationScaling;

    // Square the inputs to give finer control at low speeds
    // Math.copySign preserves the direction (+ or -) after squaring
    driveSubsystem.driveArcade(
        Math.copySign(Math.pow(Math.abs(speed), DRIVE_SENSITIVITY), speed),
        Math.copySign(Math.pow(Math.abs(rotation), ROTATION_SENSITIVITY), rotation)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}