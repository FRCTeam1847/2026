package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootCommand extends Command {

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final TurretSubsystem turretSubsystem;

  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turretSubsystem) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.turretSubsystem = turretSubsystem;
    addRequirements(shooter, indexer, turretSubsystem);
  }

  @Override
  public void execute() {

    // Shooter logic
    shooter.aimShooter();
    boolean atRPM = shooter.atSpeed(shooter.getTargetRPM(), 500);
    // Indexer logic
    if (atRPM && !turretSubsystem.isFlipping()) {
      indexer.setSpeed(-1);
      shooter.simulateShot();
    } else {
      indexer.setSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}