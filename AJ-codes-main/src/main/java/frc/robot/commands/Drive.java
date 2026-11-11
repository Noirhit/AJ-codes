package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;

public class Drive extends Command {
  CANDriveSubsystem driveSubsystem;
  CommandXboxController controller;

  // Slew rate limiters soften step changes in commanded arcade-drive outputs, which
  // reduces wheel chatter on hard stops and keeps the SparkMaxes from browning out.
  // Intentionally NOT placed in the subsystem so AutoDrive can still push raw setpoints.
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DRIVE_SLEW_PER_SEC);
  private final SlewRateLimiter zLimiter = new SlewRateLimiter(ROTATION_SLEW_PER_SEC);

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
  }

  @Override
  public void initialize() {
    // Reset limiters so stale state from a previous teleop period doesn't carry over
    xLimiter.reset(0);
    zLimiter.reset(0);
  }

  @Override
  public void execute() {
    boolean slowMode = controller.rightBumper().getAsBoolean();
    double currentDriveScaling = slowMode ? DRIVE_SLOW_SCALING : DRIVE_SCALING;
    double currentRotationScaling = slowMode ? ROTATION_SLOW_SCALING : ROTATION_SCALING;

    double speed = MathUtil.applyDeadband(-controller.getLeftY(), DEADBAND) * currentDriveScaling;
    double rotation = MathUtil.applyDeadband(-controller.getRightX(), DEADBAND) * currentRotationScaling;

    double shapedSpeed = Math.copySign(Math.pow(Math.abs(speed), DRIVE_SENSITIVITY), speed);
    double shapedRotation = Math.copySign(Math.pow(Math.abs(rotation), ROTATION_SENSITIVITY), rotation);

    driveSubsystem.driveArcade(
        xLimiter.calculate(shapedSpeed),
        zLimiter.calculate(shapedRotation));
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
