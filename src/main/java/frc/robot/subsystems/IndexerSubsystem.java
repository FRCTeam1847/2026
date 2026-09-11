package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax neo = new SparkMax(IndexerConstants.Neo_1_ID, MotorType.kBrushless);
  private final SparkMax neo2 = new SparkMax(IndexerConstants.Neo_2_ID, MotorType.kBrushless);

  public IndexerSubsystem() {
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    neoConfig
        .idleMode(IdleMode.kCoast) // set brake/coast
        .inverted(false); // set inversion if needed

    // apply configuration (reset safe parameters before applying)
    neo.configure(
        neoConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    SparkMaxConfig neoConfig2 = new SparkMaxConfig();
    neoConfig2
        .idleMode(IdleMode.kCoast) // set brake/coast
        .inverted(false); // set inversion if needed

    // apply configuration (reset safe parameters before applying)
    neo2.configure(
        neoConfig2,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters); // --- SparkMax setup
  }

  public void setSpeed(double speed) {
    neo.set(speed);
    neo2.set(speed);

  }

  public Command runIndexer(double speed) {
    return run(() -> setSpeed(speed))
        .finallyDo(interrupted -> stop());
  }

  // public Command oscillateIndexer(
  // double speed,
  // double forwardTime,
  // double reverseTime) {
  // return Commands.sequence(
  // run(() -> setSpeed(speed)).withTimeout(forwardTime),
  // runOnce(() -> stop()).withTimeout(0.25),
  // run(() -> setSpeed(-speed)).withTimeout(reverseTime),
  // runOnce(() -> stop()).withTimeout(0.25))
  // .repeatedly()
  // .finallyDo(interrupted -> stop());
  // }

  public Command stop() {
    return run(() -> {
      neo.stopMotor();
      neo2.stopMotor();
    });
  }
}
