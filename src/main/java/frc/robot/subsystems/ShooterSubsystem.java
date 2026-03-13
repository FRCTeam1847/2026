package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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

import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Supplier;

import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor;
    private final TalonFX shooterMotor2;
    private final Servo servo;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final Supplier<Pose2d> robotPoseSupplier;
    private final TurretSubsystem turretSubSystemSupplier;

    // Shooter constants
    private static final double WHEEL_RADIUS = 0.0508; // meters
    private static final double EXIT_EFFICIENCY = 0.35;
    private static final double GEAR_RATIO = 1.0;
    private static final double MAX_RPM = 5800;
    private double targetRPM = 0.0;
    private static final Translation3d SHOOTER_OFFSET = new Translation3d(0, -0.15, 0.5);

    private static final double MIN_ANGLE = 1;
    private static final double MAX_ANGLE = 140;
    private static final double SPEED_DEG_PER_SEC = 60;

    private double currentAngle = MIN_ANGLE;
    private double previousTime = 0.0;

    private boolean isDynamicHood = false;

    private static final double SHOOTER_OFFSET_Z = 0.5; // meters above floor
    private static final double TARGET_HEIGHT = 2.05; // meters
    private static final double CAMERA_HEIGHT = 0.75; // meters
    private static final double CAMERA_ANGLE_DEG = 25.0;

    private double filteredDistance = 0;

    // Limelight
    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Lookup tables
    private final TreeMap<Double, ShooterSettings> distanceLookup = new TreeMap<>();
    private final TreeMap<Double, ShooterSettings> simDistanceLookup = new TreeMap<>();

    // Simulation variables
    private final List<SimulatedBall> activeBalls = new ArrayList<>();
    private double lastSimTime = 0.0;
    private double lastShotTime = 0.0;
    private static final double SHOOT_INTERVAL = 0.25; // 4 balls/sec
    private static final double GRAVITY = -9.81;

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

    private static class ShooterSettings {
        double rpm;
        double hoodAngle;

        ShooterSettings(double rpm, double hoodAngle) {
            this.rpm = rpm;
            this.hoodAngle = hoodAngle;
        }
    }

    public ShooterSubsystem(Supplier<Pose2d> robotPoseSupplier, TurretSubsystem turretSubSystemSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.turretSubSystemSupplier = turretSubSystemSupplier;
        shooterMotor = new TalonFX(ShooterConstants.FLYWHEEL_1_Kraken_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.FLYWHEEL_2_Kraken_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterMotor.getConfigurator().apply(config);
        shooterMotor2.getConfigurator().apply(config);
        shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        servo = new Servo(ShooterConstants.HOOD_PWM_ID);
        servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

        previousTime = Timer.getFPGATimestamp();
        setHoodAngle(MIN_ANGLE);

        // --- Build example lookup table ---
        distanceLookup.put(1.0, new ShooterSettings(3500, 30));
        distanceLookup.put(2.0, new ShooterSettings(4000, 30));
        distanceLookup.put(3.0, new ShooterSettings(4500, 40));
        distanceLookup.put(4.0, new ShooterSettings(5000, 55));
        distanceLookup.put(5.0, new ShooterSettings(5500, 70));
        distanceLookup.put(6.0, new ShooterSettings(MAX_RPM, 85));

        simDistanceLookup.put(1.0, new ShooterSettings(3500, 20));
        simDistanceLookup.put(2.0, new ShooterSettings(4000, 30));
        simDistanceLookup.put(3.0, new ShooterSettings(4500, 40));
        simDistanceLookup.put(4.0, new ShooterSettings(5000, 55));
        simDistanceLookup.put(5.0, new ShooterSettings(5500, 70));
        simDistanceLookup.put(6.0, new ShooterSettings(MAX_RPM, 85));
    }

    /** ---------------- Shooter control ---------------- */
    public void setRPM(double rpm) {
        rpm = Math.min(rpm, MAX_RPM);
        targetRPM = rpm;
        double motorRPS = (rpm / 60.0) * GEAR_RATIO;
        shooterMotor.setControl(velocityRequest.withVelocity(motorRPS));
    }

    public double getRPM() {
        if (RobotBase.isSimulation()) return targetRPM;
        return (shooterMotor.getVelocity().getValueAsDouble() / GEAR_RATIO) * 60.0;
    }

    public void stop() {
        shooterMotor.set(0);
    }

    public boolean atSpeed(double targetRPM, double toleranceRPM) {
        if (RobotBase.isSimulation()) return true;
        return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /** ---------------- Hood control ---------------- */
    public void setHoodAngle(double angleDegrees) {
        angleDegrees = MathUtil.clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE);
        currentAngle = angleDegrees;
        servo.setAngle(currentAngle);
    }

    public Command dynamicHoodCommand(Supplier<Double> angleSupplier) {
        return run(() -> {
            if (isDynamicHood)
                setHoodAngle(angleSupplier.get());
        });
    }

    public void toggleDynamicHood() {
        isDynamicHood = !isDynamicHood;
    }

    public boolean isDynamicHood() {
        return isDynamicHood;
    }

    public Command raiseServo() {
        return run(() -> {
            double dt = Timer.getFPGATimestamp() - previousTime;
            previousTime = Timer.getFPGATimestamp();
            currentAngle = Math.min(MAX_ANGLE, currentAngle + SPEED_DEG_PER_SEC * dt);
            servo.setAngle(currentAngle);
        }).beforeStarting(() -> previousTime = Timer.getFPGATimestamp());
    }

    public Command lowerServo() {
        return run(() -> {
            double dt = Timer.getFPGATimestamp() - previousTime;
            previousTime = Timer.getFPGATimestamp();
            currentAngle = Math.max(MIN_ANGLE, currentAngle - SPEED_DEG_PER_SEC * dt);
            servo.setAngle(currentAngle);
        }).beforeStarting(() -> previousTime = Timer.getFPGATimestamp());
    }

    /** ---------------- Vision / distance ---------------- */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    public double getTY() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    public double getDistance() {
        if (!hasTarget()) return -1;
        double ty = getTY();
        double totalAngleRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngleRad);

        if (distance > 0) filteredDistance = 0.8 * filteredDistance + 0.2 * distance;
        return filteredDistance;
    }

    /** ---------------- Lookup tables ---------------- */
    public ShooterSettings getShooterSettingsForDistance(double distance) {
        if (distance <= 0) return distanceLookup.firstEntry().getValue();

        Map.Entry<Double, ShooterSettings> floor = distanceLookup.floorEntry(distance);
        Map.Entry<Double, ShooterSettings> ceiling = distanceLookup.ceilingEntry(distance);

        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

        double fraction = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        double rpm = floor.getValue().rpm + fraction * (ceiling.getValue().rpm - floor.getValue().rpm);
        double angle = floor.getValue().hoodAngle + fraction * (ceiling.getValue().hoodAngle - floor.getValue().hoodAngle);

        rpm = Math.min(rpm, MAX_RPM);
        return new ShooterSettings(rpm, angle);
    }

    private ShooterSettings getSimSettingsForDistance(double distance) {
        if (distance <= 0) return simDistanceLookup.firstEntry().getValue();

        Map.Entry<Double, ShooterSettings> floor = simDistanceLookup.floorEntry(distance);
        Map.Entry<Double, ShooterSettings> ceiling = simDistanceLookup.ceilingEntry(distance);

        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

        double fraction = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        double rpm = floor.getValue().rpm + fraction * (ceiling.getValue().rpm - floor.getValue().rpm);
        double angle = floor.getValue().hoodAngle + fraction * (ceiling.getValue().hoodAngle - floor.getValue().hoodAngle);

        rpm = Math.min(rpm, MAX_RPM);
        return new ShooterSettings(rpm, angle);
    }

    public void aimShooter() {
        double distance;
        if (hasTarget()) {
            distance = getDistance();
        } else if (DriverStation.isFMSAttached()) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation3d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? TurretConstants.BLUE_HUB_POSITION
                    : TurretConstants.RED_HUB_POSITION;

            double dx = hub.getX() - robotPose.getX();
            double dy = hub.getY() - robotPose.getY();
            distance = Math.hypot(dx, dy);
        } else {
            distance = distanceLookup.firstKey();
        }

        ShooterSettings settings = getShooterSettingsForDistance(distance);
        setRPM(settings.rpm);
        if (isDynamicHood) setHoodAngle(settings.hoodAngle);
    }

    /** ---------------- Simulation ---------------- */
    public double getLaunchSpeed() {
        double wheelRPM = getRPM();
        double wheelRadPerSec = (wheelRPM * 2.0 * Math.PI) / 60.0;
        return wheelRadPerSec * WHEEL_RADIUS * EXIT_EFFICIENCY;
    }

    public void simulateShot() {
        if (!Robot.isSimulation()) return;

        double now = Timer.getFPGATimestamp();
        if (now - lastShotTime < SHOOT_INTERVAL) return;
        lastShotTime = now;

        Pose2d robotPose = robotPoseSupplier.get();

        Translation2d shooterXY = SHOOTER_OFFSET.toTranslation2d()
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());
        Translation3d startPos = new Translation3d(shooterXY.getX(), shooterXY.getY(), SHOOTER_OFFSET.getZ());

        // Use sim lookup table if no target
        double distance = hasTarget() ? getDistance() : Math.hypot(
                (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? TurretConstants.BLUE_HUB_POSITION.getX()
                        : TurretConstants.RED_HUB_POSITION.getX()) - robotPose.getX(),
                (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? TurretConstants.BLUE_HUB_POSITION.getY()
                        : TurretConstants.RED_HUB_POSITION.getY()) - robotPose.getY()
        );
        ShooterSettings simSettings = getSimSettingsForDistance(distance);

        double launchSpeed = (simSettings.rpm * 2.0 * Math.PI / 60.0) * WHEEL_RADIUS * EXIT_EFFICIENCY;
        double hoodAngleRad = Math.toRadians(simSettings.hoodAngle);

        // Turret rotation + robot rotation for field-relative
        double turretYawRad = Math.toRadians(turretSubSystemSupplier.getTurretAngle()) + robotPose.getRotation().getRadians();

        double horizontalSpeed = launchSpeed * Math.cos(hoodAngleRad);
        double vx = horizontalSpeed * Math.cos(turretYawRad);
        double vy = horizontalSpeed * Math.sin(turretYawRad);
        double vz = launchSpeed * Math.sin(hoodAngleRad);

        activeBalls.add(new SimulatedBall(startPos, vx, vy, vz));
    }

    @Override
    public void simulationPeriodic() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastSimTime;
        lastSimTime = currentTime;

        Iterator<SimulatedBall> iterator = activeBalls.iterator();
        while (iterator.hasNext()) {
            SimulatedBall ball = iterator.next();
            ball.pos = new Translation3d(
                    ball.pos.getX() + ball.vx * dt,
                    ball.pos.getY() + ball.vy * dt,
                    ball.pos.getZ() + ball.vz * dt
            );
            ball.vz += GRAVITY * dt;

            if (ball.pos.getZ() <= 0) iterator.remove();
        }

        Pose3d[] ballPoses = activeBalls.stream()
                .map(b -> new Pose3d(b.pos, new Rotation3d()))
                .toArray(Pose3d[]::new);

        Logger.recordOutput("Shooter/GamePiece", ballPoses);
    }

    /** ---------------- Logging ---------------- */
    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/CurrentRPM", getRPM());
        Logger.recordOutput("Shooter/HoodAngle", currentAngle);
        Logger.recordOutput("Shooter/IsDynamicHood", isDynamicHood);
    }
}