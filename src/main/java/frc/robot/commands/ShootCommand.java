package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
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
    // 1. Always set target RPM
    double rpm = shooter.calculateFlywheelRPM();
    shooter.setRPM(rpm);

    // 2. Feed balls only if shooter is at speed
    if (shooter.atSpeed(rpm, 100)) {
      indexer.runIndexer(0.5); // feed balls
    } else {
      indexer.runIndexer(0); // stop feeding until at speed
    }

    // Optional: simulate shooting in sim
    if (RobotBase.isSimulation()) {
      shooter.simulateShot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // never finish automatically
  }
}
