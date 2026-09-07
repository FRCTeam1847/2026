// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems.V2;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;

public class ShooterSubsystem {
    // Holds and manages turret, hood and flywheel

    private final TurretSubsystemV2 turret = new TurretSubsystemV2();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final FlywheelSubsystem flywheel = new FlywheelSubsystem();

    private Supplier<AngularVelocity> flywheelVelocitySupplier = () -> DegreesPerSecond.of(0);

    // private PIDController hoodPIDController = new PIDController(10, 0, 0);
    // private PIDController turretPIDController = new PIDController(12, 0, 0);

    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        // Will make both hood and turret repeatedly try to correct themself to point at
        // the closest tag, can be reused and filtered with tag ids to get it to point
        // on the hub

        // hood.setDefaultCommand(hood.setDutyCycle(() -> {
        // var results = vision.getClosestTag();
        // if (results.isPresent()) {
        // return hoodPIDController.calculate(results.get().skew, 0);
        // }
        // return 0.0;
        // }));

        // turret.setDefaultCommand(
        //         turret.setAngle((Supplier<Angle>) () -> Degrees.of(computeHubAngle(poseSupplier))));
    }

    /** Calculate turret angle to hub using robot pose */
    public double computeHubAngle(Supplier<Pose2d> poseSupplier) {

        Pose2d pose = poseSupplier.get();

        Translation2d robot = pose.getTranslation();
        Translation2d hub = getHub().toTranslation2d();

        Translation2d diff = hub.minus(robot);

        double fieldAngle = Math.toDegrees(
                Math.atan2(diff.getY(), diff.getX()));

        double angle = fieldAngle - pose.getRotation().getDegrees();

        // Find an equivalent angle within the turret's physical range.
        while (angle < TurretConstants.REVERSE_LIMIT) {
            angle += 360.0;
        }

        while (angle > TurretConstants.FORWARD_LIMIT) {
            angle -= 360.0;
        }

        return angle;
    }

    public Translation3d getHub() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return TurretConstants.RED_HUB_POSITION;
            } else if (alliance.get() == DriverStation.Alliance.Blue) {
                return TurretConstants.BLUE_HUB_POSITION;
            }
        }

        // Not present yet; safe fallback
        return TurretConstants.BLUE_HUB_POSITION;
    }

    public Command aimAt(Angle hoodAngle, Angle turretAngle) {
        return hood.setAngle(hoodAngle).alongWith(turret.setAngle(turretAngle));
    }

    public Command runShooter() {
        if (flywheelVelocitySupplier == null) {
            DriverStation.reportWarning("Shooter velocity set to null, not running shooter", true);
            return flywheel.idle(); // Do nothing until a valid request is set
        }

        return flywheel.setVelocity(flywheelVelocitySupplier);
    }

    public Command stopShooter() {
        return flywheel.setVelocity(DegreesPerSecond.of(0));
    }

    public Command runShooter(AngularVelocity velocity) {
        if (velocity == null) {
            DriverStation.reportWarning("Shooter velocity set to null, defaulting to 0", true);
            velocity = DegreesPerSecond.of(0);
        }

        return flywheel.setVelocity(velocity);
    }

    public void setVelocitySupplier(Supplier<AngularVelocity> velocitySupplier) {
        // You would have some mathematical model for the speed of the shooter based on
        // other arguments on the field (likely distance from the hub) and use this
        // method in order to set the right shooter speed relative to that
        flywheelVelocitySupplier = velocitySupplier;
    }

    public TurretSubsystemV2 getTurret() {
        return turret;
    }

    public HoodSubsystem getHood() {
        return hood;
    }

    public FlywheelSubsystem getFlywheel() {
        return flywheel;
    }

}