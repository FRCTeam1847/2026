package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.TurretConstants;

import java.util.function.Supplier;

/**
 * Turret subsystem designed for high performance autonomous aiming.
 *
 * Features:
 * - Absolute encoder boot recovery
 * - 270° turret protection
 * - Motion Magic position control
 * - Hub tracking using robot pose
 * - Shot leading while moving
 * - Stabilization while rotating
 * - Vision micro-correction
 * - AdvantageScope 3D logging
 */
public class TurretSubsystem extends SubsystemBase {

  /* -------------------------------------------------------------------------- */
  /* Hardware */
  /* -------------------------------------------------------------------------- */

  private final TalonFX motor = new TalonFX(TurretConstants.Motor_Kraken_ID);

  /** Absolute encoder used to recover turret position after reboot */

  /** Limelight network table */
  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-shooter");

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  /* -------------------------------------------------------------------------- */
  /* Robot state suppliers */
  /* -------------------------------------------------------------------------- */

  /** Current robot pose from drivetrain */
  private Supplier<Pose2d> poseSupplier;

  /** Robot velocity from drivetrain */
  private Supplier<ChassisSpeeds> speedSupplier;

  /* -------------------------------------------------------------------------- */
  /* Persistent configuration */
  /* -------------------------------------------------------------------------- */

  private static final String ZERO_KEY = "TurretAbsoluteZero";

  /** Offset used to align absolute encoder with turret zero */
  private double absoluteOffset = 0;

  /* -------------------------------------------------------------------------- */
  /* Control state */
  /* -------------------------------------------------------------------------- */

  private double targetAngle = 0;
  private double lockedAngle = 0;
  private Translation2d lastVelocity = new Translation2d();
  private double lastTime = 0;

  public enum Mode {
    DISABLED,
    MANUAL,
    LOCK,
    TRACK_HUB,
    PASS,
    SCAN
  }

  private Mode currentMode = Mode.MANUAL;

  /* Scan test mode variables */
  private double scanAngle = -90;
  private int scanDir = 1;

  /* -------------------------------------------------------------------------- */
  /* Initialization */
  /* -------------------------------------------------------------------------- */

  public TurretSubsystem() {

    configureMotor();

    absoluteOffset = 0;

    lastTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  /** Configure TalonFX motor and Motion Magic */
  private void configureMotor() {

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    cfg.Slot0.kP = TurretConstants.kP;
    cfg.Slot0.kI = TurretConstants.kI;
    cfg.Slot0.kD = TurretConstants.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = degToMotor(TurretConstants.CRUISE_VELOCITY);

    cfg.MotionMagic.MotionMagicAcceleration = degToMotor(TurretConstants.ACCELERATION);

    cfg.MotionMagic.MotionMagicJerk = degToMotor(TurretConstants.JERK);

    motor.getConfigurator().apply(cfg);
  }

  /* -------------------------------------------------------------------------- */
  /* Setup */
  /* -------------------------------------------------------------------------- */

  /** Supply robot pose and speeds from drivetrain */
  public void setSuppliers(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds) {
    poseSupplier = pose;
    speedSupplier = speeds;
  }

  /* -------------------------------------------------------------------------- */
  /* Encoder Handling */
  /* -------------------------------------------------------------------------- */

  /** Returns turret angle from absolute encoder */

  /** Sync motor encoder with absolute encoder on boot */

  /** Returns turret angle from motor encoder */
  public double getTurretAngle() {

    return motorToDeg(
        motor.getPosition().getValueAsDouble());
  }

  /* -------------------------------------------------------------------------- */
  /* Angle Control */
  /* -------------------------------------------------------------------------- */

  /** Clamp turret angle within safe mechanical limits */
  private double clamp(double deg) {

    return Math.max(
        TurretConstants.REVERSE_LIMIT,
        Math.min(TurretConstants.FORWARD_LIMIT, deg));
  }

  /** Wrap angle between -180 and 180 */
  private double wrap(double deg) {

    while (deg > 180)
      deg -= 360;
    while (deg < -180)
      deg += 360;

    return deg;
  }

  /** Choose shortest valid path to target angle */
  private double optimize(double desired) {

    double current = getTurretAngle();

    double diff = wrap(desired - current);

    double candidate = current + diff;

    if (candidate > TurretConstants.FORWARD_LIMIT)
      candidate -= 360;

    if (candidate < TurretConstants.REVERSE_LIMIT)
      candidate += 360;

    return clamp(candidate);
  }

  /** Command turret to specific angle */
  public void setAngle(double deg) {

    // double corrected =
    // deg + TurretConstants.TURRET_FORWARD_OFFSET;

    targetAngle = optimize(deg);

    motor.setControl(
        motionMagic.withPosition(
            degToMotor(targetAngle)));
  }

  public Command increaseAngleCommand(Boolean forward) {
    return runOnce(() -> setAngle(targetAngle += forward ? 3.0 : -3.0));
  }

  /** Manual percent control */
  public void setManual(double percent) {

    percent = Math.max(-0.4, Math.min(0.4, percent));

    motor.set(percent);
  }

  /* -------------------------------------------------------------------------- */
  /* Targeting Calculations */
  /* -------------------------------------------------------------------------- */

  /** Calculate turret angle to hub using robot pose */
  private double computeHubAngle() {

    Pose2d pose = poseSupplier.get();

    Translation2d robot = pose.getTranslation();
    Translation2d hub = getHub().toTranslation2d();

    Translation2d diff = hub.minus(robot);

    double fieldAngle = Math.atan2(diff.getY(), diff.getX());

    return Math.toDegrees(fieldAngle)
        - pose.getRotation().getDegrees();
  }

  /** Stabilize turret while robot rotates */
  private double computeStabilizedHubAngle() {

    double hubAngle = computeHubAngle();

    ChassisSpeeds speeds = speedSupplier.get();

    double omegaDeg = Math.toDegrees(speeds.omegaRadiansPerSecond);

    return hubAngle +
        omegaDeg * TurretConstants.TURRET_STABILIZATION_TIME;
  }

  /** Lead shots while translating */
  private double computeLeadAngle() {
    Pose2d pose = poseSupplier.get();
    ChassisSpeeds speeds = speedSupplier.get(); // field-relative from YAGSL

    double t = TurretConstants.SHOT_LEAD_TIME;

    // Robot-center -> turret pivot offset, in robot coordinates
    Translation2d shooterOffsetRobot = TurretConstants.TURRET_OFFSET.toTranslation2d();

    // Rotate offset into field coordinates
    Translation2d shooterOffsetField = shooterOffsetRobot.rotateBy(pose.getRotation());

    // Current shooter position in field coordinates
    Translation2d shooterPos = pose.getTranslation().plus(shooterOffsetField);

    // Field-relative chassis translation velocity
    Translation2d chassisVel = new Translation2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond);

    // Extra linear velocity at shooter due to robot angular velocity
    double omega = speeds.omegaRadiansPerSecond;
    Translation2d rotationalVel = new Translation2d(
        -omega * shooterOffsetField.getY(),
        omega * shooterOffsetField.getX());

    // Total shooter velocity in field frame
    Translation2d shooterVel = chassisVel.plus(rotationalVel);

    // Predict where shooter will be when the ball reaches target
    Translation2d predictedShooterPos = shooterPos.plus(shooterVel.times(t));

    Translation2d toHub = getHub().toTranslation2d().minus(predictedShooterPos);

    double fieldAngle = Math.atan2(toHub.getY(), toHub.getX());

    return Rotation2d.fromRadians(fieldAngle)
        .minus(pose.getRotation())
        .getDegrees();
  }

  /** Limelight micro-correction */
  private double getVisionCorrection() {

    double tv = limelight.getEntry("tv").getDouble(0);

    if (tv < 1)
      return 0;

    double tx = limelight.getEntry("tx").getDouble(0);

    return -tx * TurretConstants.VISION_GAIN;
  }

  /* -------------------------------------------------------------------------- */
  /* Hub Position */
  /* -------------------------------------------------------------------------- */

  private Translation3d getHub() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        return TurretConstants.RED_HUB_POSITION;
      } else if (alliance.get() == DriverStation.Alliance.Blue) {
        return TurretConstants.BLUE_HUB_POSITION;
      }
    }

    // Not present yet; safe fallback
    return TurretConstants.BLUE_HUB_POSITION;
  }

  /* -------------------------------------------------------------------------- */
  /* Motor Conversion */
  /* -------------------------------------------------------------------------- */

  private double degToMotor(double deg) {

    return (deg / 360.0)
        * TurretConstants.MOTOR_TO_TURRET_RATIO;
    // * TurretConstants.ENCODER_TO_TURRET_RATIO; // added this
  }

  private double motorToDeg(double rot) {

    return (rot /
        TurretConstants.MOTOR_TO_TURRET_RATIO)
        // * TurretConstants.ENCODER_TO_TURRET_RATIO)) // added this
        * 360.0;
  }

  public void setMode(Mode newMode) {
    currentMode = newMode;
  }
  /* -------------------------------------------------------------------------- */
  /* Commands */
  /* -------------------------------------------------------------------------- */

  public Command lockAngle(double angle) {

    return runOnce(() -> {

      lockedAngle = angle;
      currentMode = Mode.LOCK;
    });
  }

  /* -------------------------------------------------------------------------- */
  /* Periodic Loop */
  /* -------------------------------------------------------------------------- */

  @Override
  public void periodic() {

    double turretDeg = getTurretAngle();

    /* AdvantageScope logging */

    Logger.recordOutput("Turret/AngleDeg", turretDeg);
    Logger.recordOutput("Turret/TargetDeg", targetAngle);
    Logger.recordOutput("Turret/Mode", currentMode.toString());
    Logger.recordOutput("Turret/RobotVelocity", lastVelocity);

    Pose3d turretPose = new Pose3d(
        TurretConstants.TURRET_OFFSET,
        new Rotation3d(0, 0, Math.toRadians(turretDeg)));

    Logger.recordOutput("Turret/Pose3d", turretPose);

    // if (DriverStation.isDisabled()) {

    // motor.stopMotor();
    // return;
    // }

    switch (currentMode) {

      case DISABLED:
        motor.stopMotor();
        break;

      case MANUAL:
        break;

      case LOCK:
        setAngle(lockedAngle);
        break;

      case TRACK_HUB:

        double target = computeLeadAngle() +
            getVisionCorrection();

        setAngle(target);

        break;

      case PASS:
        setAngle(computeHubAngle() + 180);
        break;

      case SCAN:
        scan();
        break;
    }
  }

  /* Test scan mode */
  private void scan() {

    scanAngle += scanDir * 2;

    if (scanAngle > TurretConstants.FORWARD_LIMIT ||
        scanAngle < TurretConstants.REVERSE_LIMIT) {

      scanDir *= -1;
    }

    setAngle(scanAngle);
  }

  @Override
  public void simulationPeriodic() {
    var simState = motor.getSimState();

    // Get the voltage Phoenix 6 wants to apply
    double appliedVolts = simState.getMotorVoltage();

    // ---- Default placeholder values for simulation ----
    double simMaxRps = 10.0; // max turret speed in rotations/sec
    // ---------------------------------------------------

    // Convert voltage to a rotation speed
    double voltsToRps = appliedVolts / 12.0; // assume 12V battery
    double rps = voltsToRps * simMaxRps;

    // Integrate for 20ms loop
    double deltaRot = rps * 0.02;

    // Update the simulated TalonFX encoder
    simState.addRotorPosition(deltaRot);
    simState.setRotorVelocity(rps);
  }

  /* get mode */
  public Mode getCurrentMode() {
    return currentMode;
  }

  /** Returns true if turret is flipping to reach the target */
  public boolean isFlipping() {
    double current = getTurretAngle();
    double diff = wrap(targetAngle - current);

    // Determine the shortest path
    boolean shortestPathCrosses180 = Math.abs(diff) > Math.abs(diff - Math.signum(diff) * 360);

    double velocityDegPerSec = motor.getVelocity().getValueAsDouble() * 360.0 / TurretConstants.MOTOR_TO_TURRET_RATIO;

    return Math.abs(diff) > 10 && Math.abs(velocityDegPerSec) > 10 || shortestPathCrosses180;
  }
}