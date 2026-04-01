package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterLaunch extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final CANFuelSubsystem fuelSubsystem;

  public ShooterLaunch(ShooterSubsystem shooterSubsystem, CANFuelSubsystem fuelSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.fuelSubsystem = fuelSubsystem;
    addRequirements(shooterSubsystem, fuelSubsystem);
  }

  @Override
  public void initialize() {
    double targetRPM = SmartDashboard.getNumber("Shooter Target RPM", SHOOTER_TARGET_RPM);
    double accelRPM = SmartDashboard.getNumber("Accelerator Target RPM", ACCELERATOR_TARGET_RPM);
    shooterSubsystem.setTargetRPM(targetRPM, accelRPM);
    fuelSubsystem.setFeederRoller(FEEDER_LAUNCH_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
