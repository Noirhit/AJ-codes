package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import java.util.function.DoubleSupplier;
import static frc.robot.Constants.FuelConstants.*;

public class SpinUp extends Command {
  private final CANFuelSubsystem fuelSubsystem;
  private final DoubleSupplier launcherRPM;

  // Fixed RPM — used for teleop with SmartDashboard values
  public SpinUp(CANFuelSubsystem fuelSubsystem) {
    this(fuelSubsystem,
        () -> SmartDashboard.getNumber(CANFuelSubsystem.KEY_LAUNCHING_LAUNCHER, LAUNCHING_LAUNCHER_RPM));
  }

  // Dynamic RPM — used when distance-based RPM is calculated first
  public SpinUp(CANFuelSubsystem fuelSubsystem, DoubleSupplier launcherRPM) {
    this.fuelSubsystem = fuelSubsystem;
    this.launcherRPM = launcherRPM;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void execute() {
    fuelSubsystem.setIntakeLauncherRoller(launcherRPM.getAsDouble());
    fuelSubsystem.setFeederRoller(
        SmartDashboard.getNumber(CANFuelSubsystem.KEY_SPINUP_FEEDER, SPIN_UP_FEEDER_RPM));
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
