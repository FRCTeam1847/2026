package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax neo = new SparkMax(IndexerConstants.Neo_1_ID, MotorType.kBrushless);
  // private final SparkMax neo2 = new SparkMax(IndexerConstants.Neo_2_ID, MotorType.kBrushless);
  // private final SparkMax neo3 = new SparkMax(IndexerConstants.Neo_3_ID, MotorType.kBrushless);
  // private final SparkMax neo2 = new SparkMax(10, MotorType.kBrushless);

  public IndexerSubsystem() {

    // --- SparkMax setup (correct API usage) ---
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    neoConfig
        .idleMode(IdleMode.kCoast) // set brake/coast
        .inverted(false); // set inversion if needed

    // apply configuration (reset safe parameters before applying)
    neo.configure(
        neoConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters); // --- SparkMax setup (correct API usage) ---

    // SparkMaxConfig neoConfig2 = new SparkMaxConfig();
    // neoConfig2
    //     .idleMode(IdleMode.kCoast) // set brake/coast
    //     .inverted(false); // set inversion if needed

    // apply configuration (reset safe parameters before applying)
    // neo2.configure(
    //     neoConfig2,
    //     com.revrobotics.ResetMode.kResetSafeParameters,
    //     com.revrobotics.PersistMode.kPersistParameters); // --- SparkMax setup (correct API usage) ---

    // SparkMaxConfig neoConfig3 = new SparkMaxConfig();
    // neoConfig3
    //     .idleMode(IdleMode.kCoast) // set brake/coast
    //     .inverted(true); // set inversion if needed

    // // apply configuration (reset safe parameters before applying)
    // neo3.configure(
    //     neoConfig3,
    //     com.revrobotics.ResetMode.kResetSafeParameters,
    //     com.revrobotics.PersistMode.kPersistParameters); // --- SparkMax setup (correct API usage) ---

  }

  /** Run both motors at the same duty cycle (-1.0 to 1.0). */
  public void setSpeed(double speed) {

    // REV SparkMax
    neo.set(speed);
    // neo2.set(speed);
    // neo3.set(speed);
    // neo2.set(speed);
  }

  public Command runIndexer(double speed) {
    return run(() -> setSpeed(speed))
        .finallyDo(interrupted -> stop());
  }

  public Command oscillateIndexer(
      double speed,
      double forwardTime,
      double reverseTime) {
    return Commands.sequence(
        run(() -> setSpeed(speed)).withTimeout(forwardTime),
        runOnce(() -> stop()).withTimeout(0.05),
        run(() -> setSpeed(-speed)).withTimeout(reverseTime),
        runOnce(() -> stop()).withTimeout(0.05))
        .repeatedly()
        .finallyDo(interrupted -> stop());
  }

  public void stop() {
    neo.stopMotor();
    // neo2.stopMotor();
    // neo3.stopMotor();
  }
}
