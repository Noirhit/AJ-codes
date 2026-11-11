package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase implements AutoCloseable {

  // ── SmartDashboard tuning keys (centralized to prevent typos across commands) ──
  public static final String KEY_INTAKING_INTAKE    = "Intaking intake roller value";
  public static final String KEY_INTAKING_FEEDER    = "Intaking feeder roller value";
  public static final String KEY_LAUNCHING_FEEDER   = "Launching feeder roller value";
  public static final String KEY_LAUNCHING_LAUNCHER = "Launching launcher roller value";
  public static final String KEY_SPINUP_FEEDER      = "Spin-up feeder roller value";
  public static final String KEY_UNSTUCK_FEEDER     = "Unstuck feeder roller value";
  public static final String KEY_FUEL_KP  = "Fuel kP";
  public static final String KEY_FUEL_KI  = "Fuel kI";
  public static final String KEY_FUEL_KD  = "Fuel kD";
  public static final String KEY_FUEL_KFF = "Fuel kFF";

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final RelativeEncoder feederEncoder;
  private final RelativeEncoder intakeLauncherEncoder;

  // Track last applied PID values so we only reconfigure when something changes
  private double lastKP  = FUEL_kP;
  private double lastKI  = FUEL_kI;
  private double lastKD  = FUEL_kD;
  private double lastKFF = FUEL_kFF;

  public CANFuelSubsystem() {
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederEncoder = feederRoller.getEncoder();
    intakeLauncherEncoder = intakeLauncherRoller.getEncoder();

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederConfig.closedLoopRampRate(FUEL_RAMP_RATE_SECONDS);
    feederConfig.closedLoop
        .pidf(FUEL_kP, FUEL_kI, FUEL_kD, FUEL_kFF)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.closedLoopRampRate(FUEL_RAMP_RATE_SECONDS);
    launcherConfig.closedLoop
        .pidf(FUEL_kP, FUEL_kI, FUEL_kD, FUEL_kFF)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber(KEY_INTAKING_FEEDER,    INTAKING_FEEDER_RPM);
    SmartDashboard.putNumber(KEY_INTAKING_INTAKE,    INTAKING_INTAKE_RPM);
    SmartDashboard.putNumber(KEY_LAUNCHING_FEEDER,   LAUNCHING_FEEDER_RPM);
    SmartDashboard.putNumber(KEY_LAUNCHING_LAUNCHER, LAUNCHING_LAUNCHER_RPM);
    SmartDashboard.putNumber(KEY_SPINUP_FEEDER,      SPIN_UP_FEEDER_RPM);
    SmartDashboard.putNumber(KEY_UNSTUCK_FEEDER,     UNSTUCK_FEEDER_RPM);

    // Live PID tuning — change these in SmartDashboard without redeploying.
    // IMPORTANT: only adjust with the fuel subsystem STOPPED to avoid control kicks.
    SmartDashboard.putNumber(KEY_FUEL_KP,  FUEL_kP);
    SmartDashboard.putNumber(KEY_FUEL_KI,  FUEL_kI);
    SmartDashboard.putNumber(KEY_FUEL_KD,  FUEL_kD);
    SmartDashboard.putNumber(KEY_FUEL_KFF, FUEL_kFF);
  }

  public void setIntakeLauncherRoller(double rpm) {
    intakeLauncherRoller.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
  }

  public void setFeederRoller(double rpm) {
    feederRoller.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
  }

  public double getFeederRPM() {
    return feederEncoder.getVelocity();
  }

  public double getLauncherRPM() {
    return intakeLauncherEncoder.getVelocity();
  }

  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  @Override
  public void close() {
    feederRoller.close();
    intakeLauncherRoller.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder Actual RPM",   getFeederRPM());
    SmartDashboard.putNumber("Launcher Actual RPM", getLauncherRPM());

    // Only reconfigure motors if any gain was changed on SmartDashboard
    double kP  = SmartDashboard.getNumber(KEY_FUEL_KP,  FUEL_kP);
    double kI  = SmartDashboard.getNumber(KEY_FUEL_KI,  FUEL_kI);
    double kD  = SmartDashboard.getNumber(KEY_FUEL_KD,  FUEL_kD);
    double kFF = SmartDashboard.getNumber(KEY_FUEL_KFF, FUEL_kFF);
    if (kP != lastKP || kI != lastKI || kD != lastKD || kFF != lastKFF) {
      // Self-contained rebuild — explicitly re-apply current limits, ramp, and feedback
      // so we don't silently lose them if someone changes ResetMode later.
      SparkMaxConfig feederConfig = new SparkMaxConfig();
      feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
      feederConfig.closedLoopRampRate(FUEL_RAMP_RATE_SECONDS);
      feederConfig.closedLoop
          .pidf(kP, kI, kD, kFF)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .outputRange(-1, 1);
      feederRoller.configure(feederConfig,
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      SparkMaxConfig launcherConfig = new SparkMaxConfig();
      launcherConfig.inverted(true);
      launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
      launcherConfig.closedLoopRampRate(FUEL_RAMP_RATE_SECONDS);
      launcherConfig.closedLoop
          .pidf(kP, kI, kD, kFF)
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .outputRange(-1, 1);
      intakeLauncherRoller.configure(launcherConfig,
          ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

      lastKP  = kP;
      lastKI  = kI;
      lastKD  = kD;
      lastKFF = kFF;
    }
  }
}
