package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor;
    private final TalonFX shooterMotor2;
    private Servo servo;

    private final Supplier<Pose2d> robotPoseSupplier;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static final double kP = 0.4;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.0;
    private static final double kV = 0.12;

    private static double targetRPM = 0.0;

    private final double minRPM = 1000;
    private final double maxRPM = 6000;

    private static final double GEAR_RATIO = 1.0;

    private double currentAngle = 1;

    private static final double MIN_ANGLE = 1;
    private static final double MAX_ANGLE = 140;

    private static final double SPEED_DEG_PER_SEC = 60;

    private double previousTime = 0.0;

    private boolean isDynamicHood = false;

    private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-shooter");

    private double filteredDistance = 0;

    private static final InterpolatingDoubleTreeMap rpmLookup = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap hoodLookup = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem(Supplier<Pose2d> robotPoseSupplier) {

        this.robotPoseSupplier = robotPoseSupplier;

        shooterMotor = new TalonFX(ShooterConstants.FLYWHEEL_1_Kraken_ID);
        shooterMotor2 = new TalonFX(ShooterConstants.FLYWHEEL_2_Kraken_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;

        shooterMotor.getConfigurator().apply(config);
        shooterMotor2.getConfigurator().apply(config);

        shooterMotor2.setControl(
                new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        servo = new Servo(ShooterConstants.HOOD_PWM_ID);
        servo.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

        previousTime = Timer.getFPGATimestamp();

        setHoodAngle(MIN_ANGLE);

        // Lookup table example values
        rpmLookup.put(1.0, 2750.0);
        rpmLookup.put(1.5, 2750.0);
        rpmLookup.put(2.0, 2750.0);
        rpmLookup.put(2.5, 3000.0);
        rpmLookup.put(3.0, 3450.0);
        rpmLookup.put(3.5, 3800.0);
        rpmLookup.put(4.0, 4050.0);
        rpmLookup.put(5.0, 4500.0);

        hoodLookup.put(1.0, 1.0);
        hoodLookup.put(1.5, 1.0);
        hoodLookup.put(2.0, 1.0);
        hoodLookup.put(2.5, 1.0);
        hoodLookup.put(3.0, 15.0);
        hoodLookup.put(3.5, 25.0);
        hoodLookup.put(4.0, 60.0);
        hoodLookup.put(5.0, 70.0);
    }

    private Translation3d getHubPosition() {

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return TurretConstants.RED_HUB_POSITION;
        }

        return TurretConstants.BLUE_HUB_POSITION;
    }

    private double getPoseDistance() {

        Pose2d robotPose = robotPoseSupplier.get();

        // Lift Pose2d to 3D with z = 0
        Translation3d robot = new Translation3d(robotPose.getX(), robotPose.getY(), 0.0);
        Translation3d hub = getHubPosition();

        // horizontal distance only
        return Math.hypot(
                robot.getX() - hub.getX(),
                robot.getY() - hub.getY());
    }

    public double getDistance() {

        if (hasTarget()) {

            double ty = getTY();

            double totalAngleRad = Math.toRadians(25.0 + ty);

            double distance = (getHubPosition().getZ() - 0.75) / Math.tan(totalAngleRad);

            if (distance > 0) {
                filteredDistance = 0.8 * filteredDistance + 0.2 * distance;
                return filteredDistance;
            }
        }

        return getPoseDistance();
    }

    public double calculateFlywheelRPM() {

        double distance = getDistance();

        double rpm = rpmLookup.get(distance);

        return MathUtil.clamp(rpm, minRPM, maxRPM);
    }

    public double calculateLaunchAngle() {

        double distance = getDistance();

        double angle = hoodLookup.get(distance);

        return MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public double getHoodAngle() {
        return currentAngle;
    }

    public void setRPM(double rpm) {

        targetRPM = rpm;

        double motorRPS = (rpm / 60.0) * GEAR_RATIO;

        shooterMotor.setControl(
                velocityRequest.withVelocity(motorRPS));
    }

    public void stop() {
        shooterMotor.set(0);
    }

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

    public boolean atSpeed(double targetRPM, double toleranceRPM) {

        if (RobotBase.isSimulation()) {
            return true;
        }

        return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
    }

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

    public Command dynamicHoodCommand(Supplier<Double> angleSupplier) {

        return run(() -> {

            if (isDynamicHood) {
                setHoodAngle(angleSupplier.get());
            }

        });
    }

    public Command raiseServo() {
        return run(() -> {
            currentAngle = MathUtil.clamp(currentAngle + 5, MIN_ANGLE, MAX_ANGLE);
            setHoodAngle(currentAngle);
        });

    }

    public Command lowerServo() {
        return run(() -> {
            currentAngle = MathUtil.clamp(currentAngle - 5, MIN_ANGLE, MAX_ANGLE);
            setHoodAngle(currentAngle);
        });

    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/CurrentRPM", getRPM());
        Logger.recordOutput("Shooter/HoodAngle", currentAngle);
        Logger.recordOutput("Shooter/IsDynamicHood", isDynamicHood);

        Logger.recordOutput("Shooter/Distance", getDistance());
    }
}
