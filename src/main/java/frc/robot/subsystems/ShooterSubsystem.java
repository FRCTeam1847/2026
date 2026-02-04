package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor;
  private Servo servo;

  // Phoenix 6 control request (reuse this object)
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // ---- TUNING VALUES ----
  private static final double kP = 0.12;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.2;
  private static final double kV = 0.12;

  // Gear ratio (motor rotations per wheel rotation)
  private static final double GEAR_RATIO = 1.0;

  public ShooterSubsystem() {
    shooterMotor = new TalonFX(10); // <-- CHANGE ID

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limiting (Kraken-safe defaults)
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // PID + Feedforward
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;

    shooterMotor.getConfigurator().apply(config);
    servo = new Servo(8);
    servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

  }

  /**
   * Set shooter speed in RPM (wheel RPM)
   */
  public void setRPM(double rpm) {
    double motorRPS = (rpm / 60.0) * GEAR_RATIO;
    shooterMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  /** Stop the shooter */
  public void stop() {
    shooterMotor.stopMotor();
  }

  /** Current shooter RPM (wheel RPM) */
  public double getRPM() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO) * 60.0;
  }

  /** True if shooter is at target speed */
  public boolean atSpeed(double targetRPM, double toleranceRPM) {
    return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
  }


  public Command zeroServo() {
    return new RunCommand(() -> servo.setAngle(1)); // 180 is full extention
  }

  public Command extendServo() {
    return new RunCommand(() -> servo.setAngle(60)); // 180 is full extention
  }
}
