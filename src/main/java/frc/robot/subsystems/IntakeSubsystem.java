package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class IntakeSubsystem extends SubsystemBase {

  private static final int ROLLER_ID = 10;
  private static final int ARM_ID = 11;

  private static final double GEAR_RATIO = 5.0;
  private static final double MIN_ANGLE = Math.toRadians(-10);
  private static final double MAX_ANGLE = Math.toRadians(120);

  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Feedforward constants
  // private static final double kS = 0.0;
  // private static final double kG = 0.5; // tune this
  // private static final double kV = 0.0;

  private static final double maxSpeed = 0.75;

  // Roller (Kraken X60)
  // private final TalonFX rollerMotor = new TalonFX(ROLLER_ID);
  private final DutyCycleOut rollerRequest = new DutyCycleOut(0);

  // Arm (NEO 1.1)
  private final SparkMax armMotor = new SparkMax(ARM_ID, MotorType.kBrushless);
  private final SparkAbsoluteEncoder absoluteEncoder;
  private final SparkClosedLoopController controller;
  private final SparkMaxConfig armConfig = new SparkMaxConfig();
  // private final ArmFeedforward feedforward;

  public IntakeSubsystem() {

    // rollerMotor.setNeutralMode(NeutralModeValue.Brake);

    controller = armMotor.getClosedLoopController();
    absoluteEncoder = armMotor.getAbsoluteEncoder();

    armConfig.encoder.positionConversionFactor(1)
    .velocityConversionFactor(1);
    
    armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(-maxSpeed, maxSpeed)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-maxSpeed, maxSpeed, ClosedLoopSlot.kSlot1).feedForward.kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  // Arm
  public void setArmPosition(double angle){
    controller.setSetpoint(angle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  // ---------------- ROLLERS ----------------

  // public void runRollers(double percent) {
  //   rollerMotor.setControl(
  //       rollerRequest.withOutput(percent));
  // }

  // public void stopRollers() {
  //   rollerMotor.stopMotor();
  // }

  @Override
  public void periodic() {
    Logger.recordOutput("IntakeArm/Angle", absoluteEncoder.getPosition());
  }
}
