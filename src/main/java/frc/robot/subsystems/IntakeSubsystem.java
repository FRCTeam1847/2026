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
  private static final double MIN_ANGLE = 120;
  private static final double MAX_ANGLE = 200;
  private static final double kP = 0.0085; // tune for your arm

  private double targetAngleDeg = 0;

  public IntakeSubsystem() {

    // ========================
    // Spark Configuration
    // ========================
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
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

    // Zero arm using absolute encoder
    armEncoder.setPosition(getAbsoluteAngle());

    // Roller motor brake mode
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    absoluteEncoder.setInverted(true);
  }

  // ========================
  // Arm Helpers
  // ========================
  public double getAngle() {
    return armEncoder.getPosition(); // preserves original encoder direction
  }

  public double getAbsoluteAngle() {
    return absoluteEncoder.get() * 360.0;
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
    rollerMotor.setControl(rollerRequest.withOutput(percent));
  }

  public void stopRollers() {
    rollerMotor.stopMotor();
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
    double angle = getAngle();

    // HARD safety stop
    if (angle < MIN_ANGLE - 10 || angle > MAX_ANGLE + 10) {
      armMotor.stopMotor();
    }

    Logger.recordOutput("IntakeArm/Angle Deg", angle);
    Logger.recordOutput("IntakeArm/Target Deg", targetAngleDeg);
    Logger.recordOutput("IntakeArm/Encoder Value", getAbsoluteAngle());
    Logger.recordOutput("IntakeArm/Rollers RPM", rollerMotor.getVelocity().getValueAsDouble() * 60.0);
    Logger.recordOutput("IntakeArm/Rollers Percent", rollerRequest.Output);
  }
}