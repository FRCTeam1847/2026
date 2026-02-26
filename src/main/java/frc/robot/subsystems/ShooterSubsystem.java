package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor;
  private final TalonFX shooterMotor2;
  private Servo servo;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final TurretSubsystem turretSubSystemSupplier;

  private static class SimulatedBall {
    Translation3d pos;
    double vx, vy, vz;

    SimulatedBall(Translation3d pos, double vx, double vy, double vz) {
      this.pos = pos;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
    }
  }

  private final List<SimulatedBall> activeBalls = new ArrayList<>();

  // Phoenix 6 control request (reuse this object)
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // ---- TUNING VALUES ----
  private static final double kP = 0.4;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.2;
  private static final double kV = 0.12;

  private static double targetRPM = 0.0;
  private final double minRPM = 1200; // fallback RPM if no target
  private final double maxRPM = 6000; // max safe RPM

  // Gear ratio (motor rotations per wheel rotation)
  private static final double GEAR_RATIO = 1.0;

  // meters
  private static final double WHEEL_RADIUS = 0.0508; // 2 inches

  // 0–1 fudge factor (slip, compression, losses)
  private static final double EXIT_EFFICIENCY = 0.5;
  private double lastShotTime = 0.0; // in seconds
  private static final double SHOOT_INTERVAL = 0.25; // 4 balls/sec

  // Shooter constants
  private static final Translation3d SHOOTER_OFFSET = new Translation3d(0, -0.15, 0.5);

  private static final double GRAVITY = -9.81; // m/s^2

  // Projectile state
  // private double simSpeed = 0.1; // 25% real speed
  private double lastSimTime = 0.0;

  // Servo variables
  private double currentAngle = 1; // start position
  private static final double MIN_ANGLE = 1;
  private static final double MAX_ANGLE = 140;
  private static final double SPEED_DEG_PER_SEC = 60; // adjust feel here
  private double previousTime = 0.0;
  private boolean isDynamicHood = false;

  // Limelight setup
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  private static final double TARGET_HEIGHT = 2.05; // meters
  private static final double CAMERA_HEIGHT = 0.75; // meters
  private static final double CAMERA_ANGLE_DEG = 25.0;

  private double filteredDistance = 0;

  public ShooterSubsystem(Supplier<Pose2d> robotPoseSupplier, TurretSubsystem turretSubSystemSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.turretSubSystemSupplier = turretSubSystemSupplier;
    shooterMotor = new TalonFX(ShooterConstants.FLYWHEEL_1_Kraken_ID);
    shooterMotor2 = new TalonFX(ShooterConstants.FLYWHEEL_2_Kraken_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limiting (Kraken-safe defaults)
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // PID + Feedforward
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    shooterMotor.getConfigurator().apply(config);
    shooterMotor2.getConfigurator().apply(config);
    shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    servo = new Servo(ShooterConstants.HOOD_PWM_ID);
    servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
    previousTime = Timer.getFPGATimestamp();
    setHoodAngle(MIN_ANGLE);
  }

  public double getLaunchSpeed() {
    double wheelRPM = getRPM();
    double wheelRadPerSec = (wheelRPM * 2.0 * Math.PI) / 60.0;
    return wheelRadPerSec * WHEEL_RADIUS * EXIT_EFFICIENCY;
  }

  public double calculateFlywheelRPM() {
    if (Robot.isSimulation()) {
      return 3500;
    }
    double distance = getDistance(); // your Limelight-based distance
    if (distance <= 0) {
      return minRPM; // fallback if Limelight has no target
    }

    // Example linear regression; tune for your robot
    double rpm = 3000 + 1000 * distance;

    // Clamp to min/max
    return Math.max(minRPM, Math.min(maxRPM, rpm));
  }

  /**
   * Set shooter speed in RPM (wheel RPM)
   */
  public void setRPM(double rpm) {
    targetRPM = rpm;
    double motorRPS = (rpm / 60.0) * GEAR_RATIO;
    shooterMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  /** Stop the shooter */
  public void stop() {
    shooterMotor.set(0);
  }

  /** Current shooter RPM (wheel RPM) */
  public double getRPM() {
    if (RobotBase.isSimulation()) {
      return targetRPM;
    }
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO) * 60.0;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  /** True if shooter is at target speed */
  public boolean atSpeed(double targetRPM, double toleranceRPM) {
    if (RobotBase.isSimulation()) {
      return true;
    }
    return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
  }

  // public Command zeroServo() {
  // return new RunCommand(() -> servo.setAngle(1)); // 180 is full extention
  // }

  // public Command extendServo() {
  // return new RunCommand(() -> servo.setAngle(140)); // 140 is our full
  // extention
  // }
  public void toggleDynamicHood() {
    isDynamicHood = !isDynamicHood;
  }

  public boolean isDynamicHood() {
    return isDynamicHood;
  }

  public void setHoodAngle(double angleDegrees) {
    angleDegrees = MathUtil.clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE);
    currentAngle = angleDegrees;
    servo.setAngle(currentAngle);
  }

  public Command raiseServo() {
    return run(() -> {
      double currentTime = Timer.getFPGATimestamp();
      double dt = currentTime - previousTime;
      previousTime = currentTime;

      currentAngle += SPEED_DEG_PER_SEC * dt;
      currentAngle = Math.min(currentAngle, MAX_ANGLE);
      servo.setAngle(currentAngle);
    }).beforeStarting(() -> {
      previousTime = Timer.getFPGATimestamp();
    });
  }

  public Command lowerServo() {
    return run(() -> {
      double currentTime = Timer.getFPGATimestamp();
      double dt = currentTime - previousTime;
      previousTime = currentTime;

      currentAngle -= SPEED_DEG_PER_SEC * dt;
      currentAngle = Math.max(currentAngle, MIN_ANGLE);
      servo.setAngle(currentAngle);
    }).beforeStarting(() -> {
      previousTime = Timer.getFPGATimestamp();
    });
  }

  public Command dynamicHoodCommand(Supplier<Double> angleSupplier) {
    return run(() -> {
      if (isDynamicHood) {
        setHoodAngle(angleSupplier.get());
      }
    });
  }

  public double getLaunchAngle() {
    // Servo angle → radians
    return Math.toRadians(20);
  }

  /** Call this to fire a simulated ball */
  public void simulateShot() {
    double now = Timer.getFPGATimestamp(); // returns seconds
    if (now - lastShotTime < SHOOT_INTERVAL) {
      return; // too soon, skip spawning
    }
    lastShotTime = now;
    Pose2d robotPose = robotPoseSupplier.get();
    double totalYaw = robotPose.getRotation().getRadians() + Math.toRadians(turretSubSystemSupplier.getTurretAngle());
    Translation2d shooterXY = SHOOTER_OFFSET.toTranslation2d()
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
    Translation3d startPos = new Translation3d(shooterXY.getX(), shooterXY.getY(), SHOOTER_OFFSET.getZ());
    Translation3d hubPos = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? turretSubSystemSupplier.BlueHubPose
        : turretSubSystemSupplier.RedHubPose;
    double launchSpeed = getLaunchSpeed();
    // 4️⃣ Compute vector from shooter to hub
    double dx = hubPos.getX() - startPos.getX();
    double dy = hubPos.getY() - startPos.getY();
    double verticalOffset = 2.0;
    double dz = (hubPos.getZ() + verticalOffset) - startPos.getZ();

    // 5️⃣ Compute pitch angle toward hub
    double launchAngle = Math.atan2(dz, Math.hypot(dx, dy));
    double horizontalSpeed = launchSpeed * Math.cos(launchAngle);
    double vx = horizontalSpeed * Math.cos(totalYaw);
    double vy = horizontalSpeed * Math.sin(totalYaw);
    double vz = launchSpeed * Math.sin(launchAngle);

    // Add new ball to the list
    activeBalls.add(new SimulatedBall(startPos, vx, vy, vz));
  }

  // #region Limelight Functions
  public boolean hasTarget() {
    return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public double getTY() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getDistance() {
    if (!hasTarget())
      return -1;

    double ty = getTY();
    double totalAngleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);
    double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngleRad);

    // Simple low-pass filter
    if (distance > 0) {
      filteredDistance = 0.8 * filteredDistance + 0.2 * distance;
    }

    return filteredDistance;
  }

  public double calculateLaunchAngle() {
    double distance = getDistance();
    if (distance <= 0)
      return 20; // fallback

    double angle = 15 + 6.5 * distance;
    return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
  }

  // #endregion
  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/CurrentRPM", getRPM());
    Logger.recordOutput("Shooter/HoodAngle", currentAngle);
    Logger.recordOutput("Shooter/IsDynamicHood", isDynamicHood);
  }

  @Override
  public void simulationPeriodic() {
    // double dt = 0.02 * simSpeed; // simulate ~50Hz scaled by simSpeed
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastSimTime;
    lastSimTime = currentTime;
    Iterator<SimulatedBall> iterator = activeBalls.iterator();

    while (iterator.hasNext()) {
      SimulatedBall ball = iterator.next();

      // Update position
      ball.pos = new Translation3d(
          ball.pos.getX() + ball.vx * dt,
          ball.pos.getY() + ball.vy * dt,
          ball.pos.getZ() + ball.vz * dt);

      // Apply gravity
      ball.vz += GRAVITY * dt;

      // Remove if hit floor
      if (ball.pos.getZ() <= 0) {
        iterator.remove();
        continue;
      }
    }

    // Log all active balls to AdvantageScope
    Pose3d[] ballPoses = activeBalls.stream()
        .map(b -> new Pose3d(b.pos, new Rotation3d()))
        .toArray(Pose3d[]::new);

    Logger.recordOutput("Shooter/GamePiece", ballPoses);
  }
}
