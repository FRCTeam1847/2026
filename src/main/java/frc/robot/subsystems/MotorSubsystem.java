// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(9);
  private final DutyCycleOut output = new DutyCycleOut(0);

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    // Optional but recommended
    motor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
  }

  public void run(double percent) {
    output.Output = percent;
    motor.setControl(output);
  }

  public void stop() {
    run(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
