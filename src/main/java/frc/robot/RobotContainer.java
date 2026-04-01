package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.AlignToNearestReef;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.Unstuck;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.FuelTrayCameraSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final FuelTrayCameraSubsystem fuelTrayCameraSubsystem = new FuelTrayCameraSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Basic Auto",
        new ExampleAuto(driveSubsystem, visionSubsystem, fuelSubsystem));
  }

  private void configureBindings() {
    // Drive default command (main2 original)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    // === OPERATOR CONTROLLER - main2 bindings preserved exactly ===
    // While the left trigger is held, intake fuel
    operatorController.leftTrigger().whileTrue(new Intake(fuelSubsystem));

    // While the right trigger is held, spin up for 1.5 seconds then launch fuel
    operatorController.rightTrigger().whileTrue(new LaunchSequence(fuelSubsystem));

    // While A is held, eject fuel back out through the intake
    operatorController.a().whileTrue(new Eject(fuelSubsystem));

    // While Y is held, reverse the launcher roller to push a stuck ball back inward
    operatorController.y().whileTrue(new Unstuck(fuelSubsystem));

    // === DRIVER CONTROLLER - new vision bindings ===
    // Right trigger: Auto-aim at nearest AprilTag
    driverController.rightTrigger().whileTrue(
        new AimAtTarget(driveSubsystem, visionSubsystem));

    // A button: Align to nearest reef scoring position
    driverController.a().whileTrue(
        new AlignToNearestReef(driveSubsystem, visionSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
