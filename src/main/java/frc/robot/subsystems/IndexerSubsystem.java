package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// CTRE Phoenix 6
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// REVLib 2025+ SparkMax API
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IndexerSubsystem extends SubsystemBase {

  private final TalonFX kraken = new TalonFX(10);
  private final DutyCycleOut krakenRequest = new DutyCycleOut(0);

  private final SparkMax neo = new SparkMax(11, MotorType.kBrushless);

  public IndexerSubsystem() {
    // --- Kraken setup ---
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kraken.getConfigurator().apply(krakenConfig);
    kraken.setNeutralMode(NeutralModeValue.Coast);

    // --- SparkMax setup (correct API usage) ---
    SparkMaxConfig neoConfig = new SparkMaxConfig();
    neoConfig
        .idleMode(IdleMode.kCoast) // set brake/coast
        .inverted(false); // set inversion if needed

    // apply configuration (reset safe parameters before applying)
    neo.configure(
        neoConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  /** Run both motors at the same duty cycle (-1.0 to 1.0). */
  public void setSpeed(double speed) {
    // CTRE Kraken
    kraken.setControl(krakenRequest.withOutput(speed));

    // REV SparkMax
    neo.set(speed);
  }

  public Command runIndexer(double speed) {
    return run(() -> setSpeed(speed))
        .finallyDo(interrupted -> stop());
  }

 
public Command oscillateIndexer(
    double speed,
    double forwardTime,
    double reverseTime
) {
  return Commands.sequence(
      run(() -> setSpeed(speed)).withTimeout(forwardTime),
      runOnce(() -> stop()).withTimeout(0.05),
      run(() -> setSpeed(-speed)).withTimeout(reverseTime),
      runOnce(() -> stop()).withTimeout(0.05)
  )
  .repeatedly()
  .finallyDo(interrupted -> stop());
}

  public void stop() {
    // setSpeed(0.0);
    kraken.setControl(krakenRequest.withOutput(0.0));

    // REV SparkMax
    neo.set(0.0);
  }
}
