package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor;
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
  private static final double kP = 0.12;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.2;
  private static final double kV = 0.12;

  private static double targetRPM = 0.0;

  // Gear ratio (motor rotations per wheel rotation)
  private static final double GEAR_RATIO = 1.0;

  // meters
  private static final double WHEEL_RADIUS = 0.0508; // 2 inches

  // 0–1 fudge factor (slip, compression, losses)
  private static final double EXIT_EFFICIENCY = 0.5;

  // Shooter constants
  private static final Translation3d SHOOTER_OFFSET = new Translation3d(0, -0.15, 0.5);

  private static final double GRAVITY = -9.81; // m/s^2

  // Projectile state
  private double simSpeed = 0.25; // 25% real speed

  public ShooterSubsystem(Supplier<Pose2d> robotPoseSupplier, TurretSubsystem turretSubSystemSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.turretSubSystemSupplier = turretSubSystemSupplier;
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

  public double getLaunchSpeed() {
    double wheelRPM = getRPM();
    double wheelRadPerSec = (wheelRPM * 2.0 * Math.PI) / 60.0;
    return wheelRadPerSec * WHEEL_RADIUS * EXIT_EFFICIENCY;
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
    shooterMotor.stopMotor();
  }

  /** Current shooter RPM (wheel RPM) */
  public double getRPM() {
    if (RobotBase.isSimulation()) {
      return targetRPM;
    }
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO) * 60.0;
  }

  /** True if shooter is at target speed */
  public boolean atSpeed(double targetRPM, double toleranceRPM) {
    if (RobotBase.isSimulation()) {
      return true;
    }
    return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
  }

  public Command zeroServo() {
    return new RunCommand(() -> servo.setAngle(1)); // 180 is full extention
  }

  public Command extendServo() {
    return new RunCommand(() -> servo.setAngle(60)); // 180 is full extention
  }

  public double getLaunchAngle() {
    // Servo angle → radians
    return Math.toRadians(20);
  }

  /** Call this to fire a simulated ball */
  public void simulateShot() {
    Pose2d robotPose = robotPoseSupplier.get();
    double totalYaw = robotPose.getRotation().getRadians() + Math.toRadians(turretSubSystemSupplier.getTurretAngle());
    Translation2d shooterXY = SHOOTER_OFFSET.toTranslation2d()
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
    Translation3d startPos = new Translation3d(shooterXY.getX(), shooterXY.getY(), SHOOTER_OFFSET.getZ());
    Translation3d hubPos = turretSubSystemSupplier.blueAlliance ? turretSubSystemSupplier.BlueHubPose
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

  @Override
  public void periodic() {
    double dt = 0.02 * simSpeed; // simulate ~50Hz scaled by simSpeed
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
