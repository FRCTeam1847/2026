package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  // ========================
  // Motors
  // ========================
  private final SparkMax armMotor = new SparkMax(IntakeConstants.ARM_Neo_ID, MotorType.kBrushless);
  private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_Kraken_ID);
  private final DutyCycleOut rollerRequest = new DutyCycleOut(0);

  // ========================
  // Encoders
  // ========================
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IntakeConstants.ARM_ENCODER_PWM_ID);
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armPID;

  // ========================
  // Constants
  // ========================
  private static final double GEAR_RATIO = 58.3;
  private static final double ABS_INSIDE_DEG = 162.0;
  private static final double ABS_OUTSIDE_DEG = 300.0;

  public static final double MIN_ANGLE = 0.0;
  public static final double MAX_ANGLE = ABS_OUTSIDE_DEG - ABS_INSIDE_DEG; // 133
  private static final double kP = 0.0085; // tune for your arm

  private double targetAngleDeg = 0;

  // ========================
  // Roller protection constants
  // ========================
  private static final double ROLLER_STATOR_LIMIT_A = 35.0; // start here, tune down/up
  private static final double ROLLER_SUPPLY_LIMIT_A = 25.0; // optional, helps brownouts
  private static final double ROLLER_MAX_STEP_PER_LOOP = 0.04; // 20ms loop -> soft ramp
  private static final double JAM_CURRENT_A = 40.0; // tune near stator limit
  private static final double JAM_VELOCITY_RPS = 3.0; // "motor commanded but barely moving"
  private static final double JAM_DETECT_TIME_S = 0.20;
  private static final double JAM_CLEAR_TIME_S = 0.12;
  private static final double JAM_CLEAR_PERCENT = 0.20;

  private double desiredRollerPercent = 0.0;
  private double appliedRollerPercent = 0.0;
  private double jamTimerSec = 0.0;
  private double jamClearTimerSec = 0.0;
  private boolean rollerJammed = false;

  public IntakeSubsystem() {

    // ========================
    // Spark Configuration
    // ========================
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(360.0 / GEAR_RATIO)
        .velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);
    config.closedLoop
        .p(kP)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    config.closedLoopRampRate(0.25).inverted(true);

    armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armEncoder = armMotor.getEncoder();
    armPID = armMotor.getClosedLoopController();

    absoluteEncoder.setInverted(true);

    // Zero arm using absolute encoder
    armEncoder.setPosition(getAbsoluteAngle());

    // Roller motor brake mode
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_LIMIT_A;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_LIMIT_A;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  // ========================
  // Arm Helpers
  // ========================
  public double getAngle() {
    return armEncoder.getPosition(); // preserves original encoder direction
  }

  public double getAbsoluteAngleRaw() {
    return absoluteEncoder.get() * 360.0;
  }

  public double getAbsoluteAngle() {
    // 0 = inside, 133 = outside
    return getAbsoluteAngleRaw() - ABS_INSIDE_DEG;
  }

  // ========================
  // Arm Control
  // ========================
  public void setTargetAngle(double degrees) {
    targetAngleDeg = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, degrees));
    armPID.setSetpoint(targetAngleDeg, ControlType.kPosition);
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - targetAngleDeg) < 2.0;
  }

  // ========================
  // Roller Control
  // ========================
  public void runRollers(double percent) {
    desiredRollerPercent = Math.max(-1.0, Math.min(1.0, percent));
  }

  public void stopRollers() {
    desiredRollerPercent = 0.0;
    jamTimerSec = 0.0;
    jamClearTimerSec = 0.0;
    rollerJammed = false;
  }

  private double slew(double current, double target, double maxDelta) {
    if (target > current + maxDelta)
      return current + maxDelta;
    if (target < current - maxDelta)
      return current - maxDelta;
    return target;
  }

  private void updateProtectedRollers() {
    double statorCurrent = Math.abs(rollerMotor.getStatorCurrent().getValueAsDouble());
    double velocityRps = Math.abs(rollerMotor.getVelocity().getValueAsDouble());

    boolean commanded = Math.abs(desiredRollerPercent) > 0.05;
    boolean jamCondition = commanded
        && velocityRps < JAM_VELOCITY_RPS
        && statorCurrent > JAM_CURRENT_A;

    if (jamClearTimerSec > 0.0) {
      jamClearTimerSec -= 0.02;
      rollerJammed = true;
    } else if (jamCondition) {
      jamTimerSec += 0.02;
      if (jamTimerSec >= JAM_DETECT_TIME_S) {
        jamClearTimerSec = JAM_CLEAR_TIME_S;
        jamTimerSec = 0.0;
        rollerJammed = true;
      }
    } else {
      jamTimerSec = 0.0;
      rollerJammed = false;
    }

    double targetPercent = desiredRollerPercent;

    // Briefly reverse to clear a jam/slip event
    if (jamClearTimerSec > 0.0 && commanded) {
      targetPercent = -Math.copySign(JAM_CLEAR_PERCENT, desiredRollerPercent);
    }

    // Soft-start / soft-change the roller output
    appliedRollerPercent = slew(appliedRollerPercent, targetPercent, ROLLER_MAX_STEP_PER_LOOP);

    rollerMotor.setControl(rollerRequest.withOutput(appliedRollerPercent));
  }

  // ========================
  // Commands
  // ========================
  public Command moveToAngleCommand(double degrees) {
    return runOnce(() -> setTargetAngle(degrees));
  }

  public Command intakeFuel() {
    return run(() -> runRollers(-0.5));
  }

  public Command throwFuel() {
    return run(() -> runRollers(0.3));
  }

  public Command stopIntakeCommand() {
    return runOnce(this::stopRollers);
  }

  public Command oscillateRollersCommand(
      double forwardTime,
      double reverseTime) {
    return Commands.sequence(
        intakeFuel().withTimeout(forwardTime),
        runOnce(() -> stopRollers()).withTimeout(0.5),
        throwFuel().withTimeout(reverseTime),
        runOnce(() -> stopRollers()).withTimeout(0.5))
        .repeatedly()
        .finallyDo(interrupted -> stopRollers());
  }

  // ========================
  // Periodic
  // ========================
  @Override
  public void periodic() {
    double absRaw = absoluteEncoder.get() * 360.0; // direct PWM reading
    double absNorm = getAbsoluteAngle(); // your adjusted arm angle
    double relAngle = getAngle(); // Spark relative encoder angle

    // HARD safety stop
    if (relAngle < MIN_ANGLE - 10 || relAngle > MAX_ANGLE + 10) {
      armMotor.stopMotor();
    }
    updateProtectedRollers();
    Logger.recordOutput("IntakeArm/Abs Raw Deg", absRaw);
    Logger.recordOutput("IntakeArm/Abs Adjusted Deg", absNorm);
    Logger.recordOutput("IntakeArm/Relative Deg", relAngle);
    Logger.recordOutput("IntakeArm/Target Deg", targetAngleDeg);
    Logger.recordOutput("IntakeArm/Abs DutyCycle", absoluteEncoder.get());

    Logger.recordOutput("IntakeArm/Rollers RPM", rollerMotor.getVelocity().getValueAsDouble() * 60.0);
    Logger.recordOutput("IntakeArm/Rollers Percent", rollerRequest.Output);
  }
}