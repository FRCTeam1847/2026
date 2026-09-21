package frc.robot.subsystems.V2;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {
  private final Servo servo = new Servo(ShooterConstants.HOOD_PWM_ID);

  /*
   * Hood exit-angle limits from CAD.
   * 25 degrees = actuator fully retracted
   * 56 degrees = actuator fully extended
   */
  private static final double MIN_HOOD_ANGLE_DEG = 25.0; // 56 actually
  private static final double MAX_HOOD_ANGLE_DEG = 56.0; // 25 actually

  /*
   * Servo command range.
   *
   * Based on testing:
   * 0 degrees = actuator minimum
   * 140 degrees = actuator maximum
   */
  private static final double MIN_SERVO_ANGLE_DEG = 0.0;
  private static final double MAX_SERVO_ANGLE_DEG = 140.0;

  // Last commanded hood angle.
  // This is NOT actual position feedback.
  private Angle commandedAngle = Degrees.of(MIN_HOOD_ANGLE_DEG);

  public HoodSubsystem() {
    servo.setBoundsMicroseconds(
        2000, // max
        1500, // deadband max
        1500, // center
        1500, // deadband min
        1000 // min
    );

    // Start at the minimum hood angle.
    setAngleDirect(commandedAngle);
  }

  /**
   * Set the hood to a desired exit angle.
   * Hood angle: 25° - 56°
   */
  public Command setAngle(Angle angle) {
    return Commands.runOnce(
        () -> setAngleDirect(angle),
        this);
  }

  /**
   * Directly set the hood angle.
   * Converts:
   * Hood angle: 34° - 65°
   * into:
   * Servo angle: 0° - 140°
   */
  public void setAngleDirect(Angle angle) {

    double hoodAngleDeg = angle.in(Degrees);

    /*
     * Prevent the requested angle from exceeding the
     * mechanical limits of the hood.
     */
    hoodAngleDeg = MathUtil.clamp(
        hoodAngleDeg,
        MIN_HOOD_ANGLE_DEG,
        MAX_HOOD_ANGLE_DEG);

    /*
     * Convert hood angle → servo angle.
     *
     * 34° hood → 0° servo
     * 65° hood → 140° servo
     */
    double servoAngleDeg = (hoodAngleDeg - MIN_HOOD_ANGLE_DEG)
        / (MAX_HOOD_ANGLE_DEG - MIN_HOOD_ANGLE_DEG)
        * (MAX_SERVO_ANGLE_DEG - MIN_SERVO_ANGLE_DEG)
        + MIN_SERVO_ANGLE_DEG;

    /*
     * Extra safety clamp on the servo command.
     */
    servoAngleDeg = MathUtil.clamp(
        servoAngleDeg,
        MIN_SERVO_ANGLE_DEG,
        MAX_SERVO_ANGLE_DEG);
    commandedAngle = Degrees.of(hoodAngleDeg);

    /*
     * Send the PWM command to the linear servo.
     */
    servo.setAngle(servoAngleDeg);
  }

  /**
   * Set the hood angle from a Supplier.
   * Useful for commands where the desired angle is continuously
   * calculated, such as shoot-on-the-move.
   */
  public Command setAngle(Supplier<Angle> angleSupplier) {
    return Commands.run(
        () -> setAngleDirect(angleSupplier.get()),
        this);
  }

  /**
   * Returns the last commanded hood angle.
   *
   * IMPORTANT:
   * The WCP-0408 has no position feedback, so this is the
   * requested position, NOT the measured physical position.
   */
  public Angle getAngle() {
    return commandedAngle;
  }

  // /**
  // * Directly command the servo using its 0-140 degree range.
  // *
  // * This is useful for testing/calibration.
  // */
  // public Command setServoAngle(Supplier<Double> servoAngleSupplier) {
  // return Commands.run(
  // () -> setServoAngleDirect(servoAngleSupplier.get()),
  // this);
  // }

  // /**
  // * Directly command the servo using its 0-140 degree range.
  // */
  // private void setServoAngleDirect(double servoAngle) {

  // servoAngle = MathUtil.clamp(
  // servoAngle,
  // MIN_SERVO_ANGLE_DEG,
  // MAX_SERVO_ANGLE_DEG);

  // servo.setAngle(servoAngle);
  // }
}