// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.V2.FlywheelSubsystem;
import frc.robot.subsystems.V2.HoodSubsystem;
import frc.robot.subsystems.V2.ShooterSubsystem;
import frc.robot.subsystems.V2.TurretSubsystemV2;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import java.util.List;
import java.util.function.Supplier;

/**
 * Largely written by Eeshwar based off their blog at
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends Command {

  // Subsystems
  private final TurretSubsystemV2 turretSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  private static final Translation3d TurretOffset = new Translation3d(-.2, .10, 0.3);

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

  public ShootOnTheMoveCommand(
      ShooterSubsystem shooterSubsystem,
      SwerveSubsystem swerve,
      Supplier<Pose2d> currentPose,
      Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
      Pose2d goal) {

    turretSubsystem = shooterSubsystem.getTurret();
    hoodSubsystem = shooterSubsystem.getHood();
    flywheelSubsystem = shooterSubsystem.getFlywheel();

    robotPose = currentPose;
    swerveSubsystem = swerve;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;

    // Test Results
    for (var entry : List.of(
        Pair.of(Meters.of(1), RPM.of(1300)),
        Pair.of(Meters.of(2), RPM.of(1300)),
        Pair.of(Meters.of(3), RPM.of(1500)),
        Pair.of(Meters.of(4), RPM.of(1500)),
        Pair.of(Meters.of(5), RPM.of(1700)))) {

      shooterTable.put(
          entry.getFirst().in(Meters),
          entry.getSecond().in(RPM));
    }

    setName("Shoot on the move");
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // -------------------------------------------------------
    // 1. GET ROBOT VELOCITY
    // -------------------------------------------------------

    var robotSpeed = fieldOrientedChassisSpeeds.get();

    // -------------------------------------------------------
    // 2. LATENCY COMPENSATION
    // -------------------------------------------------------

    Translation2d futurePos = robotPose.get().getTranslation().plus(
        new Translation2d(
            robotSpeed.vxMetersPerSecond,
            robotSpeed.vyMetersPerSecond)
            .times(latency));

    // -------------------------------------------------------
    // 3. GET TARGET VECTOR
    // -------------------------------------------------------

    Translation2d goalLocation = goalPose.getTranslation();

    Translation2d targetVec = goalLocation.minus(futurePos);

    double dist = targetVec.getNorm();

    // Prevent division by zero
    if (dist < 0.01) {
      return;
    }

    // -------------------------------------------------------
    // 4. GET BASE RPM FROM YOUR SHOOTER TABLE
    // -------------------------------------------------------

    double idealRPM = shooterTable.get(dist);

    // -------------------------------------------------------
    // 5. CONVERT TABLE RPM → LINEAR WHEEL VELOCITY
    //
    // The table remains RPM.
    // We only convert it to m/s for the physics calculation.
    // -------------------------------------------------------

    double wheelCircumference = FLYWHEEL_DIAMETER.in(Meters) * Math.PI;

    double idealExitVelocity = idealRPM * wheelCircumference / 60.0;

    // -------------------------------------------------------
    // 6. VECTOR SUBTRACTION
    // -------------------------------------------------------

    Translation2d robotVelVec = new Translation2d(
        robotSpeed.vxMetersPerSecond,
        robotSpeed.vyMetersPerSecond);

    Translation2d shotVec = targetVec.div(dist)
        .times(idealExitVelocity)
        .minus(robotVelVec);

    // -------------------------------------------------------
    // 7. CALCULATE TURRET ANGLE
    // -------------------------------------------------------

    // shotVec is FIELD-relative.
    // Convert it to an angle relative to the robot.

    Rotation2d shotDirection = shotVec.getAngle();

    Rotation2d robotRotation = robotPose.get().getRotation();

    Rotation2d turretRotation = shotDirection.minus(robotRotation);

    double turretAngle = turretRotation.getDegrees();

    // -------------------------------------------------------
    // 8. CALCULATE REQUIRED HORIZONTAL VELOCITY
    // -------------------------------------------------------

    double horizontalVelocity = shotVec.getNorm();

    // -------------------------------------------------------
    // 9. CALCULATE HOOD ANGLE
    // -------------------------------------------------------

    double heightDifference = HUB_HEIGHT.in(Meters) - SHOOTER_HEIGHT.in(Meters);

    double gravity = 9.80665;

    // Calculate the vertical velocity required to reach the hub.
    double verticalVelocity = (gravity * dist * dist
        + 2.0 * heightDifference
            * horizontalVelocity * horizontalVelocity)
        / (2.0 * dist * horizontalVelocity);

    // Calculate launch angle above horizontal.
    double newPitchDegrees = Math.toDegrees(
        Math.atan2(
            verticalVelocity,
            horizontalVelocity));

    // -------------------------------------------------------
    // 10. CONVERT REQUIRED EXIT VELOCITY → RPM
    // -------------------------------------------------------

    double newRPM = horizontalVelocity
        / wheelCircumference
        * 60.0;

    // -------------------------------------------------------
    // 11. SET OUTPUTS
    // -------------------------------------------------------

    DriverStation.reportWarning(
        String.format(
            "Distance: %.2f m | Table RPM: %.0f | SOTM RPM: %.0f | Hood: %.2f° | Turret: %.2f°",
            dist,
            idealRPM,
            newRPM,
            newPitchDegrees,
            turretAngle),
        false);

    turretSubsystem.setAngleDirect(
        Degrees.of(turretAngle));

    hoodSubsystem.setAngleDirect(
        Radians.of(newPitchDegrees));

    flywheelSubsystem.setRPMDirect(
        MetersPerSecond.of(
            newRPM * wheelCircumference / 60.0));

    // -------------------------------------------------------
    // 12. SIMULATION
    // -------------------------------------------------------

    Rotation2d shooterRotation = robotPose.get()
        .getRotation()
        .plus(Rotation2d.fromDegrees(turretAngle));

    Translation2d shooterOffset = TurretOffset.toTranslation2d()
        .rotateBy(shooterRotation);

    RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
        swerveSubsystem.getPose()
            .getTranslation()
            .plus(shooterOffset),
        shooterOffset,
        swerveSubsystem.getRobotVelocity(),
        shooterRotation,
        SHOOTER_HEIGHT,
        MetersPerSecond.of(
            newRPM * wheelCircumference / 60.0),
        hoodSubsystem.getAngle());

    fuelOnFly
        .disableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance()
        .addGamePieceProjectile(fuelOnFly);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}