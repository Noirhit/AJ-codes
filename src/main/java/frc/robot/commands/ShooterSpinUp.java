package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSpinUp extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public ShooterSpinUp(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    double targetRPM = SmartDashboard.getNumber("Shooter Target RPM", SHOOTER_TARGET_RPM);
    double accelRPM = SmartDashboard.getNumber("Accelerator Target RPM", ACCELERATOR_TARGET_RPM);
    shooterSubsystem.setTargetRPM(targetRPM, accelRPM);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooterSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isAtSpeed();
  }
}
