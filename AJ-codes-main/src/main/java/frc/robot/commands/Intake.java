package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

public class Intake extends Command {
  CANFuelSubsystem fuelSubsystem;

  public Intake(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    fuelSubsystem.setIntakeLauncherRoller(
        SmartDashboard.getNumber(CANFuelSubsystem.KEY_INTAKING_INTAKE, INTAKING_INTAKE_RPM));
    fuelSubsystem.setFeederRoller(
        SmartDashboard.getNumber(CANFuelSubsystem.KEY_INTAKING_FEEDER, INTAKING_FEEDER_RPM));
  }

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
