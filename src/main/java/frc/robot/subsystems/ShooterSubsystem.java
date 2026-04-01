package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase implements AutoCloseable {
  private final SparkMax upperFlywheel;
  private final SparkMax lowerFlywheel;
  private final SparkMax acceleratorFlywheel;

  private final RelativeEncoder upperEncoder;
  private final RelativeEncoder lowerEncoder;
  private final RelativeEncoder acceleratorEncoder;

  private double targetRPM = 0.0;
  private double acceleratorTargetRPM = 0.0;

  public ShooterSubsystem() {
    upperFlywheel = new SparkMax(UPPER_FLYWHEEL_ID, MotorType.kBrushless);
    lowerFlywheel = new SparkMax(LOWER_FLYWHEEL_ID, MotorType.kBrushless);
    acceleratorFlywheel = new SparkMax(ACCELERATOR_ID, MotorType.kBrushless);

    upperEncoder = upperFlywheel.getEncoder();
    lowerEncoder = lowerFlywheel.getEncoder();
    acceleratorEncoder = acceleratorFlywheel.getEncoder();

    SparkMaxConfig upperConfig = new SparkMaxConfig();
    upperConfig.smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    upperConfig.idleMode(IdleMode.kCoast);
    upperConfig.inverted(true);
    upperConfig.closedLoop
        .pidf(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kFF)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    upperFlywheel.configure(upperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig lowerConfig = new SparkMaxConfig();
    lowerConfig.smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    lowerConfig.idleMode(IdleMode.kCoast);
    lowerConfig.closedLoop
        .pidf(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kFF)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    lowerFlywheel.configure(lowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig acceleratorConfig = new SparkMaxConfig();
    acceleratorConfig.smartCurrentLimit(FLYWHEEL_CURRENT_LIMIT);
    acceleratorConfig.idleMode(IdleMode.kCoast);
    acceleratorConfig.closedLoop
        .pidf(SHOOTER_kP, SHOOTER_kI, SHOOTER_kD, SHOOTER_kFF)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    acceleratorFlywheel.configure(acceleratorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter Target RPM", SHOOTER_TARGET_RPM);
    SmartDashboard.putNumber("Accelerator Target RPM", ACCELERATOR_TARGET_RPM);
  }

  public void setTargetRPM(double shooterRPM, double accelRPM) {
    this.targetRPM = shooterRPM;
    this.acceleratorTargetRPM = accelRPM;
    upperFlywheel.getClosedLoopController().setReference(shooterRPM,
        ControlType.kVelocity);
    lowerFlywheel.getClosedLoopController().setReference(shooterRPM,
        ControlType.kVelocity);
    acceleratorFlywheel.getClosedLoopController().setReference(accelRPM,
        ControlType.kVelocity);
  }

  public void setTargetRPM(double shooterRPM) {
    setTargetRPM(shooterRPM, shooterRPM * 0.5);
  }

  public void stop() {
    targetRPM = 0.0;
    acceleratorTargetRPM = 0.0;
    upperFlywheel.set(0);
    lowerFlywheel.set(0);
    acceleratorFlywheel.set(0);
  }

  public boolean isAtSpeed() {
    return Math.abs(getUpperRPM() - targetRPM) < SHOOTER_TOLERANCE_RPM
        && Math.abs(getLowerRPM() - targetRPM) < SHOOTER_TOLERANCE_RPM;
  }

  public double getUpperRPM() {
    return upperEncoder.getVelocity();
  }

  public double getLowerRPM() {
    return lowerEncoder.getVelocity();
  }

  public double getAcceleratorRPM() {
    return acceleratorEncoder.getVelocity();
  }

  @Override
  public void close() {
    upperFlywheel.close();
    lowerFlywheel.close();
    acceleratorFlywheel.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Upper RPM", getUpperRPM());
    SmartDashboard.putNumber("Shooter Lower RPM", getLowerRPM());
    SmartDashboard.putNumber("Shooter Accelerator RPM", getAcceleratorRPM());
    SmartDashboard.putBoolean("Shooter At Speed", isAtSpeed());
  }
}
