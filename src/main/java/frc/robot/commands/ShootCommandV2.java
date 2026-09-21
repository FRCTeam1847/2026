package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.V2.ShooterSubsystem;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShootCommandV2 extends Command {

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final SwerveSubsystem swerveSubsystem;
  private static final Translation3d TurretOffset = new Translation3d(-.2, .10, 0.3);

  // Time between fuel pieces being fired in simulation.
  // private static final double SIM_FUEL_SHOT_DELAY = 0.3;
  private static final double SIM_FUEL_BALLS_PER_SECOND = 5.0;
  private static final double SIM_FUEL_SHOT_INTERVAL = 1.0 / SIM_FUEL_BALLS_PER_SECOND;

  private final Timer fuelShotTimer = new Timer();
  private static final double RPM_TOLERANCE = 100; // RPM
  // private static final double ANGLE_TOLERANCE = 2; // degrees

  public ShootCommandV2(ShooterSubsystem shooter, IndexerSubsystem indexer, SwerveSubsystem swerveSubsystem) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.swerveSubsystem = swerveSubsystem;
    // addRequirements(indexer);
  }

  @Override
  public void initialize() {
    fuelShotTimer.restart();
  }

  @Override
  public void execute() {

    // continuously update shooter target based on current robot position /
    // limelight
    // double targetRPM = shooter.calculateFlywheelRPM();
    // double targetHood = shooter.calculateLaunchAngle();

    // shooter.setRPM(targetRPM);
    // shooter.setHoodAngle(targetHood);

    // // only feed balls if shooter is at speed and hood is near target
    boolean rpmGood = shooter.getFlywheel().atSpeed(RPM_TOLERANCE);
    // boolean hoodGood = Math.abs(shooter.getHoodAngle() - targetHood) <=
    // ANGLE_TOLERANCE;

    if (rpmGood) {
      indexer.setSpeed(IndexerConstants.INDEXER_SPEED);
    } else {
      indexer.setSpeed(0);
    }

    if (RobotBase.isSimulation()
        && fuelShotTimer.hasElapsed(SIM_FUEL_SHOT_INTERVAL) && rpmGood) {

      fuelShotTimer.restart();
      Rotation2d shooterRotation = swerveSubsystem.getPose()
          .getRotation()
          .plus(Rotation2d.fromDegrees(shooter.getTurret().getTurretAngle().in(Degrees)));
      Translation2d shooterOffset = TurretOffset.toTranslation2d()
          .rotateBy(shooterRotation);
      RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
          swerveSubsystem.getPose()
              .getTranslation()
              .plus(shooterOffset),
          shooterOffset,
          swerveSubsystem.getRobotVelocity(),
          shooterRotation,
          Meters.of(0.45),
          shooter.getFlywheel().getSurfaceVelocity().div(2),
          shooter.getHood().getAngle());

      fuelOnFly
          .disableBecomesGamePieceOnFieldAfterTouchGround();

      SimulatedArena.getInstance()
          .addGamePieceProjectile(fuelOnFly);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setSpeed(0);
    fuelShotTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}