package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends Command {
  private final ShooterSubsystem shooter;
  private final double rpm;

  public SpinUpShooter(ShooterSubsystem shooter, double rpm) {
    this.shooter = shooter;
    this.rpm = rpm;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setRPM(rpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
