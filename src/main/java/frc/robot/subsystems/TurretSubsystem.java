package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.networktables.*;

import frc.robot.Constants.TurretConstants;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turretMotor = new TalonFX(TurretConstants.Motor_Kraken_ID);

  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(TurretConstants.Encoder_PWM_ID);

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private static final String ZERO_KEY = "TurretAbsoluteZero";

  private double absoluteZeroOffset = 0;

  private double targetDeg = 0;

  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<ChassisSpeeds> chassisSpeedSupplier;

  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-shooter");

  private final Translation3d turretOffset = new Translation3d(
      TurretConstants.CAMERA_OFFSET_X,
      TurretConstants.CAMERA_OFFSET_Y,
      TurretConstants.CAMERA_OFFSET_Z);

  public enum TurretMode {
    DISABLED,
    MANUAL,
    HOLD_ANGLE,
    FIELD_LOCK,
    TAG_TRACK,
    AUTO_AIM,
    PASS_SHOT,
    SCAN
  }

  private TurretMode mode = TurretMode.DISABLED;

  private double holdAngle = 0;

  private double scanAngle = -90;
  private int scanDir = 1;

  public TurretSubsystem() {

    configureMotor();

    absoluteZeroOffset = Preferences.getDouble(ZERO_KEY, 0);

    zeroToAbsolute();
  }

  private void configureMotor() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.Slot0.kP = TurretConstants.kP;
    cfg.Slot0.kI = TurretConstants.kI;
    cfg.Slot0.kD = TurretConstants.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = degreesToMotorRotations(
        TurretConstants.CRUISE_VELOCITY);

    cfg.MotionMagic.MotionMagicAcceleration = degreesToMotorRotations(
        TurretConstants.ACCELERATION);

    cfg.MotionMagic.MotionMagicJerk = degreesToMotorRotations(
        TurretConstants.JERK);

    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(
        TurretConstants.FORWARD_LIMIT);

    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(
        TurretConstants.REVERSE_LIMIT);

    turretMotor.getConfigurator().apply(cfg);
  }

  public void setSuppliers(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds) {

    robotPoseSupplier = pose;
    chassisSpeedSupplier = speeds;
  }

  public void setMode(TurretMode newMode) {
    mode = newMode;
  }

  public Command setModeCommand(TurretMode newMode) {
    return runOnce(() -> mode = newMode);
  }

  public void calibrateAbsoluteZero() {

    absoluteZeroOffset = absoluteEncoder.get();

    Preferences.setDouble(ZERO_KEY, absoluteZeroOffset);
  }

  public void zeroToAbsolute() {

    turretMotor.setPosition(
        degreesToMotorRotations(
            getAbsoluteTurretAngle()));
  }

  public double getAbsoluteTurretAngle() {

    double raw = absoluteEncoder.get();

    double adjusted = raw - absoluteZeroOffset;

    adjusted = (adjusted % 1 + 1) % 1;

    double encoderDeg = adjusted * 360;

    return encoderDeg /
        TurretConstants.ENCODER_TO_TURRET_RATIO;
  }

  public double getTurretAngle() {

    return motorRotationsToDegrees(
        turretMotor.getPosition()
            .getValueAsDouble());
  }

  public void setManualPercent(double percent) {

    percent = Math.max(
        -TurretConstants.MAX_MANUAL_PERCENT,
        Math.min(
            TurretConstants.MAX_MANUAL_PERCENT,
            percent));

    turretMotor.set(percent);
  }

  public void setAngle(double deg) {

    targetDeg = optimizeRotation(deg);

    turretMotor.setControl(
        motionMagic.withPosition(
            degreesToMotorRotations(targetDeg)));
  }

  private double computeHubAngle() {

    Pose2d pose = robotPoseSupplier.get();

    Translation2d robot = pose.getTranslation();

    Translation3d hub3d = getTargetHub();

    Translation2d hub = new Translation2d(hub3d.getX(), hub3d.getY());

    Translation2d diff = hub.minus(robot);

    double fieldAngle = Math.atan2(diff.getY(), diff.getX());

    return Math.toDegrees(fieldAngle)
        - pose.getRotation().getDegrees();
  }

  private double computeLeadAngle() {

    Pose2d pose = robotPoseSupplier.get();

    ChassisSpeeds speeds = chassisSpeedSupplier.get();

    Translation2d velocity = new Translation2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond);

    Translation2d future = pose.getTranslation()
        .plus(
            velocity.times(
                TurretConstants.SHOT_LEAD_TIME));

    Translation3d hub3d = getTargetHub();

    Translation2d hub = new Translation2d(hub3d.getX(), hub3d.getY());

    Translation2d diff = hub.minus(future);

    double fieldAngle = Math.atan2(diff.getY(), diff.getX());

    return Math.toDegrees(fieldAngle)
        - pose.getRotation().getDegrees();
  }

  private void trackAprilTag() {

    double tv = limelight.getEntry("tv").getDouble(0);

    if (tv < 1) {
      setAngle(computeHubAngle());
      return;
    }

    double tx = limelight.getEntry("tx").getDouble(0);

    setAngle(getTurretAngle() - tx);
  }

  private void scan() {

    scanAngle += scanDir * TurretConstants.SCAN_SPEED;

    if (scanAngle > TurretConstants.FORWARD_LIMIT ||
        scanAngle < TurretConstants.REVERSE_LIMIT) {

      scanDir *= -1;
    }

    setAngle(scanAngle);
  }

  private void updateLimelightPose() {

    double turretDeg = getTurretAngle();

    Rotation3d rot = new Rotation3d(
        0,
        0,
        Math.toRadians(turretDeg));

    Pose3d pose = new Pose3d(turretOffset, rot);

    limelight
        .getEntry("camerapose_robotspace")
        .setDoubleArray(
            new double[] {
                pose.getX(),
                pose.getY(),
                pose.getZ(),
                pose.getRotation().getX(),
                pose.getRotation().getY(),
                pose.getRotation().getZ()
            });
  }

  private double optimizeRotation(double desired) {

    double current = getTurretAngle();

    double diff = wrap(desired - current);

    double candidate = current + diff;

    if (candidate > TurretConstants.FORWARD_LIMIT)
      candidate -= 360;

    if (candidate < TurretConstants.REVERSE_LIMIT)
      candidate += 360;

    return candidate;
  }

  private double wrap(double deg) {

    while (deg > 180)
      deg -= 360;

    while (deg < -180)
      deg += 360;

    return deg;
  }

  private double degreesToMotorRotations(double deg) {

    return (deg / 360.0) *
        TurretConstants.MOTOR_TO_TURRET_RATIO;
  }

  private double motorRotationsToDegrees(double rot) {

    return (rot /
        TurretConstants.MOTOR_TO_TURRET_RATIO) *
        360.0;
  }

  private Translation3d getTargetHub() {

    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {

      if (alliance.get() == DriverStation.Alliance.Red) {
        return TurretConstants.RED_HUB_POSITION;
      }

      if (alliance.get() == DriverStation.Alliance.Blue) {
        return TurretConstants.BLUE_HUB_POSITION;
      }
    }

    return TurretConstants.BLUE_HUB_POSITION;
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      turretMotor.stopMotor();
      return;
    }

    updateLimelightPose();

    switch (mode) {

      case DISABLED:
        turretMotor.stopMotor();
        break;

      case MANUAL:
        break;

      case HOLD_ANGLE:
        setAngle(holdAngle);
        break;

      case FIELD_LOCK:
        setAngle(computeHubAngle());
        break;

      case TAG_TRACK:
        trackAprilTag();
        break;

      case AUTO_AIM:
        setAngle(computeLeadAngle());
        break;

      case PASS_SHOT:
        setAngle(computeHubAngle() + 180);
        break;

      case SCAN:
        scan();
        break;
    }

    Logger.recordOutput(
        "Turret/AngleDeg",
        getTurretAngle());

    Logger.recordOutput(
        "Turret/AbsoluteAngle",
        getAbsoluteTurretAngle());

    Logger.recordOutput(
        "Turret/Mode",
        mode.toString());
    Logger.recordOutput(
        "Turret/TargetHub",
        getTargetHub());
  }
}