package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

  // Gear ratios
  private static final double MOTOR_TO_TURRET_RATIO = 48.0;
  private static final double ENCODER_TO_TURRET_RATIO = 10.0;

  public Translation3d BlueHubPose = new Translation3d(4.597, 4.035, 1.575);
  public Translation3d RedHubPose = new Translation3d(11.938, 4.035, 1.575);
  public boolean blueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

  // Wrap-around tracking for absolute encoder
  private double lastRaw = 0.0;
  private double wrapTurns = 0.0;

  // Turret position offset on robot (meters)
  private final Translation3d turretOffset = new Translation3d(0, -0.15, 0.46);

  public TurretSubsystem() {
    configureMotor();
    // No setDutyCycleRange needed; raw 0–1 × 360 gives degrees
  }

  private void configureMotor() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Software limits in motor rotations
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(180.0);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(-180.0);

    motor.getConfigurator().apply(cfg);
  }

  // --- Public API ---

  public void setPercent(double percent) {
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
    // Phoenix 6 TalonFX.getPosition() -> StatusSignal<Angle>
    double motorRotations = motor.getPosition().getValueAsDouble(); // rotations
    return motorRotationsToDegrees(motorRotations);
  }

  public double getAbsoluteTurretAngle() {
    double raw = absoluteEncoder.get(); // 0.0–1.0
    double delta = raw - lastRaw;
    if (delta > 0.5)
      wrapTurns--;
    if (delta < -0.5)
      wrapTurns++;
    lastRaw = raw;

    double degrees = raw * 360.0; // convert normalized PWM to degrees
    double totalDegrees = degrees + wrapTurns * 360.0;
    return wrapDegrees(totalDegrees / ENCODER_TO_TURRET_RATIO);
  }

  // --- Conversion helpers ---

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

  public void aimAtHub(Pose2d robotPose) {
    Pose3d hubPose = new Pose3d(
        blueAlliance ? BlueHubPose : RedHubPose,
        new Rotation3d(0, 0, 0));

    // 5️⃣ Compute turret-relative angle
    Translation2d turretFieldXY = turretOffset.toTranslation2d()
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());

    Translation2d hubXY = hubPose.getTranslation().toTranslation2d();

    double dx = hubXY.getX() - turretFieldXY.getX();
    double dy = hubXY.getY() - turretFieldXY.getY();

    double targetYawField = Math.atan2(dy, dx);

    double turretRelativeRad = targetYawField - robotPose.getRotation().getRadians();
    double turretRelativeDeg = Math.toDegrees(turretRelativeRad);
    turretRelativeDeg = wrapDegrees(turretRelativeDeg);

    setAngle(turretRelativeDeg);
  }

  // --- Periodic logging for AdvantageScope 3D ---
  @Override
  public void periodic() {
    double turretDeg = getTurretAngle();
    double absDeg = getAbsoluteTurretAngle();
    double turretRad = Math.toRadians(turretDeg);

    // Turret rotation around vertical axis
    Rotation3d turretRotation = new Rotation3d(0, 0, turretRad);
    Pose3d turretPose = new Pose3d(turretOffset, turretRotation);
    // Log 3D pose
    Logger.recordOutput("Turret/Pose3d", turretPose);

    // Log raw values
    Logger.recordOutput("Turret/AngleDeg", turretDeg);
    Logger.recordOutput("Turret/AbsoluteDeg", absDeg);
    Logger.recordOutput("Turret/EncoderRaw", absoluteEncoder.get());
    Logger.recordOutput("Turret/MotorRotations", motor.getPosition().getValueAsDouble());
  }
}
