package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private static final double RPM_TOLERANCE = 200; // RPM
  private static final double ANGLE_TOLERANCE = 5; // degrees

  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
  }

  @Override
  public void execute() {

    // continuously update shooter target based on current robot position /
    // limelight
    double targetRPM = shooter.calculateFlywheelRPM();
    double targetHood = shooter.calculateLaunchAngle();

    shooter.setRPM(targetRPM);
    shooter.setHoodAngle(targetHood);

    // only feed balls if shooter is at speed and hood is near target
    boolean rpmGood = shooter.atSpeed(shooter.getTargetRPM(), RPM_TOLERANCE);
    boolean hoodGood = Math.abs(shooter.getHoodAngle() - targetHood) <= ANGLE_TOLERANCE;

    if (rpmGood && hoodGood) {
      indexer.setSpeed(-0.8);
    } else {
      indexer.setSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}