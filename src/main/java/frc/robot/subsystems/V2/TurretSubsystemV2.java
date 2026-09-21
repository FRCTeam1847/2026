// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems.V2;

import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;
import yams.motorcontrollers.SmartMotorControllerConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystemV2 extends SubsystemBase {
        private final TalonFX turretMotor = new TalonFX(TurretConstants.Motor_Kraken_ID);
        private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(4, 0, 0)
                        // .withTrapezoidalProfile(DegreesPerSecond.of(180),
                        // DegreesPerSecondPerSecond.of(90))
                        .withTrapezoidalProfile(// This makes it faster. Above is the old slow one
                                        DegreesPerSecond.of(720),
                                        DegreesPerSecondPerSecond.of(1440))
                        .withGearing(new MechanismGearing(
                                        GearBox.fromReductionStages(TurretConstants.MOTOR_TO_TURRET_RATIO)))
                        .withIdleMode(MotorMode.BRAKE)
                        .withMotorInverted(false)
                        // Setup Telemetry
                        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                        // Power Optimization
                        .withStatorCurrentLimit(Amps.of(40))
                        .withClosedLoopRampRate(Seconds.of(0.25))
                        .withOpenLoopRampRate(Seconds.of(0.25))
                        .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
                        // .withContinuousWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled
                        // bc the pivot can
                        // spin infinitely
                        .withMomentOfInertia(Meters.of(0.25), Pounds.of(4)); // MOI Calculation
        private final SmartMotorController turretSMC = new TalonFXWrapper(turretMotor,
                        DCMotor.getKrakenX60(1),
                        motorConfig);

        private final PivotConfig turretConfig = new PivotConfig()
                        .withHardLimits(
                                        Degrees.of(TurretConstants.REVERSE_LIMIT),
                                        Degrees.of(TurretConstants.FORWARD_LIMIT))
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH); // Telemetry

        private final Pivot turret = new Pivot(turretConfig, turretSMC);

        private void sysIdDrive(Voltage voltage) {
                turretSMC.setVoltage(voltage);
        }

        public void sysIdLLog(SysIdRoutineLog log) {
                log.motor("Turret")
                                .voltage(turretMotor.getMotorVoltage().getValue())
                                .angularPosition(turretSMC.getMechanismPosition())
                                .angularVelocity(turretSMC.getMechanismVelocity());
        }

        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(this::sysIdDrive, this::sysIdLLog, this, "Turret"));

        public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
                return sysIdRoutine.quasistatic(direction).onlyWhile(RobotState::isTest);
        }

        public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
                return sysIdRoutine.dynamic(direction).onlyWhile(RobotState::isTest);
        }

        public TurretSubsystemV2() {
        }

        public Command setTurretAngleCommand(Supplier<Angle> angleSupplier) {
                return turret.setAngle(angleSupplier);
        }

        public Command setTurrentAngleCommand(Angle angle) {
                return turret.setAngle(angle).onlyWhile(RobotState::isTest);
        }

        public Angle getTurretAngle() {
                return turret.getAngle();
        }

        @Override
        public void periodic() {
                turret.updateTelemetry();
                Pose3d turretPose = new Pose3d(
                                TurretConstants.TURRET_OFFSET,
                                new Rotation3d(0, 0, getTurretAngle().in(Radians)));

                Logger.recordOutput("Turret/Pose3d", turretPose);
        }

        @Override
        public void simulationPeriodic() {
                turret.simIterate();
        }
}