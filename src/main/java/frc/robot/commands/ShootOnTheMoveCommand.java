// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.V2.ShooterSubsystem;
import java.util.List;
import java.util.function.Supplier;

/**
 * Shoot-on-the-move command using empirically tuned lookup tables for
 * hood angle and stationary flywheel RPM.
 *
 * The hood and flywheel values should be tuned at known distances while
 * the robot is stationary. Robot velocity is then compensated separately
 * for shooting while moving.
 *
 * Based on:
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends ParallelCommandGroup {
        private static final Distance FLYWHEEL_DIAMETER = Inches.of(4);
        private final Supplier<Pose2d> robotPose;
        /**
         * Current field-oriented chassis speeds.
         */
        private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;

        private final Pose2d goalPose;

        /**
         * Time in seconds between when the robot is told to move
         * and when the shooter actually shoots.
         */
        private static final double LATENCY_SECONDS = 0.15;

        /**
         * Minimum distance represented by the lookup tables.
         */
        private static final double MIN_SHOOTING_DISTANCE_METERS = 1.0;

        /**
         * Maximum distance represented by the lookup tables.
         */
        private static final double MAX_SHOOTING_DISTANCE_METERS = 5.0;

        /**
         * Maps distance in meters to stationary flywheel RPM.
         * These values should be experimentally tuned.
         */
        private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

        /**
         * Maps distance in meters to hood angle in degrees.
         * These values should be experimentally tuned.
         */
        private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

        /**
         * Current calculated turret setpoint.
         */
        private double turretAngleDegrees = 0.0;

        /**
         * Current calculated hood setpoint.
         */
        private double hoodAngleDegrees = 0.0;

        /**
         * Current calculated flywheel setpoint.
         * This is the final compensated exit velocity in meters/second.
         */
        private double flywheelRPM = 0.0;

        public ShootOnTheMoveCommand(
                        ShooterSubsystem shooterSubsystem,
                        SwerveSubsystem swerve,
                        Supplier<Pose2d> currentPose,
                        Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                        Pose2d goal) {

                robotPose = currentPose;
                this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
                this.goalPose = goal;
                /*
                 * SHOOTER LOOKUP TABLE
                 */
                for (var entry : List.of(
                                Pair.of(Meters.of(1.0), RPM.of(2500)),
                                Pair.of(Meters.of(2.0), RPM.of(2750)),
                                Pair.of(Meters.of(3.0), RPM.of(3300)),
                                Pair.of(Meters.of(4.0), RPM.of(4500)),
                                Pair.of(Meters.of(5.0), RPM.of(4800)))) {

                        shooterTable.put(
                                        entry.getFirst().in(Meters),
                                        entry.getSecond().in(RPM));
                }

                /*
                 * HOOD LOOKUP TABLE
                 */
                for (var entry : List.of(
                                Pair.of(Meters.of(1.0), 25.0),
                                Pair.of(Meters.of(2.0), 25.0),
                                Pair.of(Meters.of(3.0), 25.0),
                                Pair.of(Meters.of(4.0), 25.0),
                                Pair.of(Meters.of(5.0), 25.0))) {

                        hoodTable.put(
                                        entry.getFirst().in(Meters),
                                        entry.getSecond());
                }

                /*
                 * Calculate the desired turret, hood, and flywheel setpoints.
                 *
                 * The mechanism commands consume these values through
                 * suppliers, so they are updated every scheduler cycle.
                 */
                Command calculateShot = Commands.run(this::calculateShot);
                Command turretCommand = shooterSubsystem.getTurret()
                                .setTurretAngleCommand(() -> Degrees.of(turretAngleDegrees));
                Command hoodCommand = shooterSubsystem.getHood()
                                .setAngle(() -> Degrees.of(hoodAngleDegrees));
                Command flywheelCommand = shooterSubsystem.getFlywheel()
                                .setRPMSupplier(
                                                () -> RPM.of(flywheelRPM));
                addCommands(calculateShot, turretCommand, hoodCommand, flywheelCommand);
                setName("Shoot on the move");
        }

        private void calculateShot() {
                // 1. GET ROBOT VELOCITY

                ChassisSpeeds robotSpeed = fieldOrientedChassisSpeeds.get();

                Translation2d robotVelocity = new Translation2d(
                                robotSpeed.vxMetersPerSecond,
                                robotSpeed.vyMetersPerSecond);

                // 2. LATENCY COMPENSATION

                Pose2d currentPose = robotPose.get();

                Translation2d futurePosition = currentPose
                                .getTranslation()
                                .plus(robotVelocity.times(LATENCY_SECONDS));

                // 3. GET TARGET VECTOR

                Translation2d goalLocation = goalPose.getTranslation();

                Translation2d targetVector = goalLocation.minus(futurePosition);

                double distance = targetVector.getNorm();

                if (distance < 0.01) {
                        return;
                }

                /*
                 * Clamp the lookup distance to the range we have actually
                 * characterized.
                 *
                 * This prevents the shooter from requesting nonsense values
                 * when the robot gets closer/farther than the tested range.
                 */

                double lookupDistance = clamp(
                                distance,
                                MIN_SHOOTING_DISTANCE_METERS,
                                MAX_SHOOTING_DISTANCE_METERS);

                Translation2d targetDirection = targetVector.div(distance);

                // 4. GET EMPIRICAL SHOOTER RPM

                /*
                 * This is the RPM that was experimentally determined
                 * for this distance while stationary.
                 */

                double baseRPM = shooterTable.get(lookupDistance);

                // 5. GET EMPIRICAL HOOD ANGLE

                hoodAngleDegrees = hoodTable.get(lookupDistance);

                // 6. RPM -> BALL/EXIT VELOCITY

                double wheelCircumference = FLYWHEEL_DIAMETER.in(Meters) * Math.PI;

                double baseExitVelocity = baseRPM * wheelCircumference / 60.0;

                // 7. SHOOT-ON-THE-MOVE VELOCITY COMPENSATION

                /*
                 * The ball's field velocity needs to point toward the target.
                 *
                 * Ball field velocity =
                 * robot velocity
                 * + ball velocity relative to robot
                 *
                 * Therefore:
                 *
                 * ball velocity relative to robot =
                 * desired field velocity
                 * - robot velocity
                 */

                Translation2d desiredFieldVelocity = targetDirection.times(baseExitVelocity);

                Translation2d compensatedShotVelocity = desiredFieldVelocity.minus(robotVelocity);

                // 8. CALCULATE TURRET ANGLE

                Rotation2d shotDirection = compensatedShotVelocity.getAngle();

                Rotation2d turretRotation = shotDirection.minus(currentPose.getRotation());

                turretAngleDegrees = turretRotation.getDegrees();

                // 9. CALCULATE COMPENSATED FLYWHEEL RPM

                /*
                 * Convert the compensated exit velocity back into
                 * flywheel RPM.
                 */
                double compensatedExitVelocity = compensatedShotVelocity.getNorm();

                flywheelRPM = (compensatedExitVelocity * 60.0)
                                / wheelCircumference;

                // 10. DEBUG

                // DriverStation.reportWarning(
                //                 String.format(
                //                                 "Distance: %.2f m | "
                //                                                 + "Lookup: %.2f m | "
                //                                                 + "Base RPM: %.0f | "
                //                                                 + "Compensated RPM: %.0f | "
                //                                                 + "Exit Velocity: %.2f m/s | "
                //                                                 + "Hood: %.2f° | "
                //                                                 + "Turret: %.2f°",

                //                                 distance,
                //                                 lookupDistance,
                //                                 baseRPM,
                //                                 flywheelRPM,
                //                                 compensatedExitVelocity,
                //                                 hoodAngleDegrees,
                //                                 turretAngleDegrees),
                //                 false);

        }

        /**
         * Clamps a value between a minimum and maximum.
         */
        private static double clamp(
                        double value,
                        double minimum,
                        double maximum) {

                return Math.max(minimum, Math.min(maximum, value));
        }
}