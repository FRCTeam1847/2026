package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {

  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
  }

  @Override
  public void execute() {

    // Shooter logic
    double rpm = shooter.calculateFlywheelRPM();
    shooter.setRPM(rpm);
    boolean atRPM = shooter.atSpeed(rpm, 500);
    // Indexer logic
    if (atRPM) {
      indexer.setSpeed(-1);
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