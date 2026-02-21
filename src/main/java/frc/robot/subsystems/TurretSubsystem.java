package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(9);
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(0);

  private static final double MOTOR_TO_TURRET_RATIO = 48.0;
  private static final double ENCODER_TO_TURRET_RATIO = 10.0;

  private static final String ZERO_KEY = "TurretAbsoluteZero";
  private double absoluteZeroOffset = 0.0;

  public Translation3d BlueHubPose = new Translation3d(4.597, 4.035, 1.575);
  public Translation3d RedHubPose = new Translation3d(11.938, 4.035, 1.575);

  private final Translation3d turretOffset = new Translation3d(0, -0.15, 0.46);

  // Safe movement parameters
  private static final double MAX_PERCENT_OUTPUT = 0.25; // 25% max power
  private static final double AIM_DEADBAND_DEGREES = 5.0; // stop if within 1°

  public TurretSubsystem() {
    configureMotor();
    absoluteZeroOffset = Preferences.getDouble(ZERO_KEY, 0.0);
    zeroToAbsolute();
  }

  private void configureMotor() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(130.0);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(-130.0);
    motor.getConfigurator().apply(cfg);
  }

  public void setPercent(double percent) {
    // Clamp output
    if (percent > MAX_PERCENT_OUTPUT)
      percent = MAX_PERCENT_OUTPUT;
    if (percent < -MAX_PERCENT_OUTPUT)
      percent = -MAX_PERCENT_OUTPUT;
    motor.set(percent);
  }

  public void setAngle(double degrees) {
    motor.setPosition(degreesToMotorRotations(degrees));
  }

  public void zeroToAbsolute() {
    double turretDeg = getAbsoluteTurretAngle();
    motor.setPosition(degreesToMotorRotations(turretDeg));
  }

  public double getTurretAngle() {
    return motorRotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public double getAbsoluteTurretAngle() {
    double raw = absoluteEncoder.get();
    double adjusted = raw - absoluteZeroOffset;
    adjusted = (adjusted % 1.0 + 1.0) % 1.0;
    double encoderDegrees = adjusted * 360.0;
    double turretDegrees = encoderDegrees / ENCODER_TO_TURRET_RATIO;
    return turretDegrees;
  }

  private double degreesToMotorRotations(double turretDegrees) {
    return (turretDegrees / 360.0) * MOTOR_TO_TURRET_RATIO;
  }

  private double motorRotationsToDegrees(double motorRotations) {
    return (motorRotations / MOTOR_TO_TURRET_RATIO) * 360.0;
  }

  private double wrapDegrees(double deg) {
    while (deg > 180.0)
      deg -= 360.0;
    while (deg < -180.0)
      deg += 360.0;
    return deg;
  }

  private Alliance getAllianceSafe() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  public void aimAtHub(Pose2d robotPose) {
    // 1. Pick the correct hub
    Translation3d hubPose3d = (getAllianceSafe() == Alliance.Blue) ? BlueHubPose : RedHubPose;
    Translation2d hubXY = hubPose3d.toTranslation2d();

    // 2. Turret position in field coordinates
    Translation2d turretFieldXY = turretOffset.toTranslation2d()
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());

    // 3. Compute angle to hub
    double dx = hubXY.getX() - turretFieldXY.getX();
    double dy = hubXY.getY() - turretFieldXY.getY();
    double targetYawRad = Math.atan2(dy, dx);
    double targetYawDeg = Math.toDegrees(targetYawRad);

    // 4. Compute relative angle from turret
    double turretRelativeDeg = wrapDegrees(targetYawDeg - Math.toDegrees(robotPose.getRotation().getRadians()));

    // 5. Compute shortest path delta
    double currentDeg = getTurretAngle();
    double delta = turretRelativeDeg - currentDeg;

    // // 6. Only move if outside deadband
    // if (Math.abs(delta) > AIM_DEADBAND_DEGREES) {
    // double percentOutput = MAX_PERCENT_OUTPUT * Math.signum(delta);
    // setPercent(percentOutput);
    // } else {
    // setPercent(0.0);
    // }

    // 6. Smooth proportional control
    double kP = 0.01; // Start here. Tune later.

    if (Math.abs(delta) > AIM_DEADBAND_DEGREES) {

      double output = kP * delta;

      // Clamp to max output
      if (output > MAX_PERCENT_OUTPUT)
        output = MAX_PERCENT_OUTPUT;
      if (output < -MAX_PERCENT_OUTPUT)
        output = -MAX_PERCENT_OUTPUT;

      setPercent(output);

    } else {
      setPercent(0.0);
    }
  }

  @Override
  public void periodic() {
    double turretDeg = getTurretAngle();
    double absDeg = getAbsoluteTurretAngle();
    double turretRad = Math.toRadians(turretDeg);

    Rotation3d turretRotation = new Rotation3d(0, 0, turretRad);
    Pose3d turretPose = new Pose3d(turretOffset, turretRotation);

    Logger.recordOutput("Turret/Pose3d", turretPose);
    Logger.recordOutput("Turret/AngleDeg", turretDeg);
    Logger.recordOutput("Turret/AbsoluteDeg", absDeg);
    Logger.recordOutput("Turret/EncoderRaw", absoluteEncoder.get());
    Logger.recordOutput("Turret/MotorRotations", motor.getPosition().getValueAsDouble());
  }
}
