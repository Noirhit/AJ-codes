package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class AutoDrive extends Command {
  CANDriveSubsystem driveSubsystem;
  double xSpeed, zRotation;

  public AutoDrive(CANDriveSubsystem driveSystem, double xSpeed, double zRotation) {
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    driveSubsystem.driveArcade(xSpeed, zRotation);
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
