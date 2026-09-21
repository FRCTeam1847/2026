// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems.V2;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_1_Kraken_ID);
  private final TalonFX flywheelMotor2 = new TalonFX(ShooterConstants.FLYWHEEL_2_Kraken_ID);
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.3, 0.0, 0.01) // from sim // KP = 0.3, KD = 0.01, Ki=0.0
      .withTrapezoidalProfile(
          RPM.of(5500), // MAX RPM
          RotationsPerSecondPerSecond.of(183))
      .withVelocityTrapezoidalProfile(true)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // 1:1
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40)) // 40-60 Amps?
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25)) // optional
      .withOpenLoopRampRate(Seconds.of(0.25)) // optional
      .withFeedforward(new SimpleMotorFeedforward(0.0102, 0.1145, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.12, 0.113, 0.0))
      .withMomentOfInertia(Inches.of(4), Pounds.of(1.69))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new TalonFXWrapper(flywheelMotor, DCMotor.getKrakenX60(2), motorConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig, motor);

  public FlywheelSubsystem() {
    flywheelMotor2.setControl(
        new Follower(ShooterConstants.FLYWHEEL_1_Kraken_ID, MotorAlignmentValue.Opposed));
  }

  private AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  private Distance getFlywheelDiamater() {
    return Inches.of(4);
  }

  public LinearVelocity getSurfaceVelocity() {
    return MetersPerSecond.of(getVelocity().in(RotationsPerSecond) * getFlywheelDiamater().times(Math.PI).in(Meters));
  }

  public boolean atSpeed(double tolorence) {
    double speed = getVelocity().baseUnitMagnitude();
    double targetSpeed = flywheel.getMechanismSetpointVelocity().get().baseUnitMagnitude();
    System.out.println(String.format("Speed: %.2f RPM | "
        + "Target Speed: %.2f RPM | ",
        speed, targetSpeed, tolorence));
    return Math.abs(speed - targetSpeed) <= tolorence;
  }

  /** Command that sets rpm */
  public Command setRPMSupplier(Supplier<AngularVelocity> speed) {
    return flywheel.run(speed);
  }

  public Command setRPMCommand(AngularVelocity speed) {
    return flywheel.run(speed);
  }

  @Override
  public void periodic() {
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }

}