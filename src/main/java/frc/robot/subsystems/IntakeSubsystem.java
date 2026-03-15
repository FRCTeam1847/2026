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
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {

  // Motors
  private final SparkMax armMotor = new SparkMax(IntakeConstants.ARM_Neo_ID, MotorType.kBrushless);
  private final SparkMax armMotor2 = new SparkMax(IntakeConstants.ARM_2_Neo_ID, MotorType.kBrushless);

  private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_Kraken_ID);

  private final DutyCycleOut rollerRequest = new DutyCycleOut(0);

  // Absolute encoder
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IntakeConstants.ARM_ENCODER_PWM_ID);

  // Spark objects
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armPID;

  private static final double GEAR_RATIO = 45.0;

  private double targetAngleDeg = 45;

  public IntakeSubsystem() {

    // Spark configuration
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);

    config.encoder
        .positionConversionFactor(360.0 / GEAR_RATIO)
        .velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);

    config.closedLoop
        .p(0.04)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    armMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Spark configuration
    SparkMaxConfig config2 = new SparkMaxConfig();

    config2.idleMode(IdleMode.kCoast);

    config2.encoder
        .positionConversionFactor(360.0 / GEAR_RATIO)
        .velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);

    config.closedLoop
        .p(0.04)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    config2.follow(IntakeConstants.ARM_Neo_ID,true);

    armMotor2.configure(
        config2,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Get encoder + PID controller
    armEncoder = armMotor.getEncoder();
    armPID = armMotor.getClosedLoopController();

    // Zero arm using absolute encoder
    double absoluteAngle = absoluteEncoder.get() * 360.0;
    armEncoder.setPosition(absoluteAngle);

    // Roller motor
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // Arm control
  public void setTargetAngle(double degrees) {
    targetAngleDeg = degrees;
    armPID.setSetpoint(degrees, ControlType.kPosition);
  }

  public double getAngle() {
    return armEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - targetAngleDeg) < 2;
  }

  // Rollers
  public void runRollers(double percent) {
    rollerMotor.setControl(rollerRequest.withOutput(percent));
  }

  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  public double getAbsoluteAngle() {
    return absoluteEncoder.get() * 360.0;
  }

  // Commands
  public Command moveToAngle(double degrees) {
    return runOnce(() -> setTargetAngle(degrees));
  }

  public Command intakeFuel() {
    return run(() -> runRollers(-0.4));
  }

  public Command stopIntakeCommand() {
    return runOnce(this::stopRollers);
  }
  // ========================
  // Periodic
  // ========================

  @Override
  public void periodic() {

    Logger.recordOutput("IntakeArm/Relative Angle Deg", getAngle());
    Logger.recordOutput("IntakeArm/Absolute Angle Deg", getAbsoluteAngle());
    Logger.recordOutput("IntakeArm/Target Deg", targetAngleDeg);

    Logger.recordOutput(
        "IntakeArm/Rollers/RPM",
        rollerMotor.getVelocity().getValueAsDouble() * 60.0);

    Logger.recordOutput(
        "IntakeArm/Rollers/Percent",
        rollerRequest.Output);
  }
}