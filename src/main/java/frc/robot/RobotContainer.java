package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants;

import frc.robot.commands.AimAtTarget;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.Unstuck;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.FuelTrayCameraSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  // ── Subsystems ──────────────────────────────────────────────────────────────
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final FuelTrayCameraSubsystem fuelTrayCameraSubsystem = new FuelTrayCameraSubsystem();

  // ── Controllers ─────────────────────────────────────────────────────────────
  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  // ── Auto Chooser ─────────────────────────────────────────────────────────────
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureAutoChooser();
  }

  // ── Button Bindings
  private void configureBindings() {

    // DRIVER: Default command — arcade drive with turbo bumpers
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    // DRIVER: Right trigger — auto-aim at hub tag based on alliance (Red: 4, Blue: 25)
    driverController.rightTrigger().whileTrue(
        new AimAtTarget(driveSubsystem, visionSubsystem, VisionSubsystem.getAutoTargetTagId()));

    // ── OPERATOR CONTROLLER

    // Left trigger held  → intake fuel balls
    operatorController.leftTrigger().whileTrue(
        new Intake(fuelSubsystem));

    // Right trigger held → teleop shot at LAUNCHING_LAUNCHER_RPM (-3700)
    operatorController.rightTrigger().whileTrue(
        new LaunchSequence(fuelSubsystem));

    // A held → eject fuel back out through the intake
    operatorController.a().whileTrue(
        new Eject(fuelSubsystem));

    // Y held → reverse rollers to free a stuck ball (tunable via "Unstuck feeder roller value")
    operatorController.y().whileTrue(
        new Unstuck(fuelSubsystem));
  }

  //Autonomous Chooser
  private void configureAutoChooser() {

    // DEFAULT: shoot all 8 preloaded balls for the full 15s auto period.
    // Uses distance-based RPM lookup via LaunchSequence(fuel, vision); falls back to
    // the fixed LAUNCHING_LAUNCHER_RPM if no hub tag is visible at start.
    autoChooser.setDefaultOption(
        "🔵 Shoot 8 Balls (15s)",
        new LaunchSequence(fuelSubsystem, FuelConstants.AUTO_LAUNCHING_LAUNCHER_RPM).withTimeout(15.0));

    // SAFE fallback: drive forward off the line if the fuel system fails at the field.
    autoChooser.addOption(
        "🟢 [SAFE] Drive Forward Only",
        new AutoDrive(driveSubsystem, 0.4, 0).withTimeout(2));

    // Do nothing — sometimes correct if alliance partners need the space.
    autoChooser.addOption(
        "🟡 Do Nothing",
        new WaitCommand(15));

    // Put chooser on SmartDashboard — select your auto BEFORE the match starts!
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // ── Called by Robot.java to get the selected auto command ───────────────────
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}