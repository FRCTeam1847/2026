// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
 * Largely written by Eeshwar based off their blog at
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends ParallelCommandGroup {

        private static final Distance SHOOTER_HEIGHT = Meters.of(0.45);

        private static final Distance HUB_HEIGHT = Meters.of(2.64);

        // Same wheel diameter used by FlywheelSubsystem
        private static final Distance FLYWHEEL_DIAMETER = Inches.of(4);

        /**
         * Current robot pose. (Blue-alliance)
         */
        private final Supplier<Pose2d> robotPose;

        /**
         * Current field-oriented chassis speeds.
         */
        private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;

        /**
         * Pose to shoot at.
         */
        private final Pose2d goalPose;

        /**
         * Time in seconds between when the robot is told to move
         * and when the shooter actually shoots.
         */
        private final double latency = 0.15;

        /**
         * Maps Distance to RPM.
         */
        private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

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
         */
        private double flywheelVelocity = 0.0;

        public ShootOnTheMoveCommand(
                        ShooterSubsystem shooterSubsystem,
                        SwerveSubsystem swerve,
                        Supplier<Pose2d> currentPose,
                        Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                        Pose2d goal) {

                robotPose = currentPose;
                this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
                this.goalPose = goal;

                // Test Results
                for (var entry : List.of(
                                Pair.of(Meters.of(1), RPM.of(800)),
                                Pair.of(Meters.of(2), RPM.of(1000)),
                                Pair.of(Meters.of(3), RPM.of(1500)),
                                Pair.of(Meters.of(4), RPM.of(1500)),
                                Pair.of(Meters.of(5), RPM.of(1700)))) {

                        shooterTable.put(
                                        entry.getFirst().in(Meters),
                                        entry.getSecond().in(RPM));
                }

                /*
                 * Calculate the desired turret, hood, and flywheel setpoints.
                 *
                 * The mechanism commands below consume these values through
                 * suppliers. They are therefore updated every scheduler cycle
                 * without this command directly calling subsystem setpoint methods.
                 */
                Command calculateShot = Commands.run(this::calculateShot);

                Command turretCommand = shooterSubsystem.getTurret().setAngle(() -> Degrees.of(turretAngleDegrees));

                Command hoodCommand = shooterSubsystem.getHood().setAngle(() -> Degrees.of(hoodAngleDegrees));

                Command flywheelCommand = shooterSubsystem.getFlywheel()
                                .setRPM(() -> MetersPerSecond.of(flywheelVelocity));

                addCommands(calculateShot,
                                turretCommand, hoodCommand, flywheelCommand);

                setName("Shoot on the move");
        }

        private void calculateShot() {

                // -------------------------------------------------------
                // 1. GET ROBOT VELOCITY
                // -------------------------------------------------------

                ChassisSpeeds robotSpeed = fieldOrientedChassisSpeeds.get();
                Translation2d robotVelocity = new Translation2d(robotSpeed.vxMetersPerSecond,
                                robotSpeed.vyMetersPerSecond);

                // -------------------------------------------------------
                // 2. LATENCY COMPENSATION
                // -------------------------------------------------------

                Pose2d currentPose = robotPose.get();
                Translation2d futurePosition = currentPose.getTranslation().plus(robotVelocity.times(latency));

                // -------------------------------------------------------
                // 3. GET TARGET VECTOR
                // -------------------------------------------------------

                Translation2d goalLocation = goalPose.getTranslation();

                Translation2d targetVector = goalLocation.minus(futurePosition);
                double distance = targetVector.getNorm();

                if (distance < 0.01) {
                        return;
                }

                Translation2d targetDirection = targetVector.div(distance);

                // -------------------------------------------------------
                // 4. GET BASE RPM FROM SHOOTER TABLE
                // -------------------------------------------------------

                double baseRPM = shooterTable.get(distance);

                // -------------------------------------------------------
                // 5. RPM -> BALL/EXIT VELOCITY
                //
                // This conversion is only necessary because vector
                // compensation requires a linear velocity.
                // -------------------------------------------------------

                double wheelCircumference = FLYWHEEL_DIAMETER.in(Meters) * Math.PI;
                double baseExitVelocity = baseRPM * wheelCircumference / 60.0;

                // -------------------------------------------------------
                // 6. SHOOT-ON-THE-MOVE VELOCITY COMPENSATION
                //
                // Ball field velocity must point toward the target.
                //
                // Ball field velocity = robot velocity + ball velocity
                // relative to robot.
                //
                // Therefore:
                //
                // ball velocity relative to robot =
                // desired field velocity - robot velocity
                // -------------------------------------------------------

                Translation2d desiredFieldVelocity = targetDirection.times(baseExitVelocity);
                Translation2d compensatedShotVelocity = desiredFieldVelocity.minus(robotVelocity);

                // -------------------------------------------------------
                // 7. CALCULATE TURRET ANGLE
                // -------------------------------------------------------

                Rotation2d shotDirection = compensatedShotVelocity.getAngle();
                Rotation2d turretRotation = shotDirection.minus(currentPose.getRotation());

                turretAngleDegrees = turretRotation.getDegrees();

                // -------------------------------------------------------
                // 8. CALCULATE REQUIRED HORIZONTAL VELOCITY
                // -------------------------------------------------------

                double horizontalVelocity = compensatedShotVelocity.getNorm();

                // -------------------------------------------------------
                // 9. CALCULATE HOOD ANGLE
                // -------------------------------------------------------

                double heightDifference = HUB_HEIGHT.in(Meters) - SHOOTER_HEIGHT.in(Meters);
                double gravity = 9.80665;

                /*
                 * Calculate the required vertical velocity.
                 *
                 * Given:
                 *
                 * x = horizontalVelocity * t
                 * y = verticalVelocity * t - 0.5 * g * t^2
                 *
                 * Solving for verticalVelocity gives:
                 */

                double verticalVelocity = (heightDifference
                                + gravity * distance * distance / (2.0 * horizontalVelocity * horizontalVelocity))
                                * horizontalVelocity / distance;

                // -------------------------------------------------------
                // 10. CALCULATE HOOD / LAUNCH ANGLE
                // -------------------------------------------------------

                hoodAngleDegrees = Math.toDegrees(Math.atan2(verticalVelocity, horizontalVelocity));

                // -------------------------------------------------------
                // 11. CALCULATE TOTAL REQUIRED EXIT VELOCITY
                //
                // The flywheel needs to produce the TOTAL launch speed,
                // not just the horizontal component.
                // -------------------------------------------------------

                double requiredExitVelocity = Math.hypot(horizontalVelocity, verticalVelocity);

                // -------------------------------------------------------
                // 13. UPDATE FLYWHEEL SETPOINT
                // -------------------------------------------------------
                flywheelVelocity = requiredExitVelocity;

                DriverStation.reportWarning(
                                String.format(
                                                "Distance: %.2f m | "
                                                                + "requiredExitVelocity: %.0f | Hood: %.2f° | "
                                                                + "Turret: %.2f°",
                                                distance,
                                                requiredExitVelocity,
                                                hoodAngleDegrees,
                                                turretAngleDegrees),
                                false);
        }
}