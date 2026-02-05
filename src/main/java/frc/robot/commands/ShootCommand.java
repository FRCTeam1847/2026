package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {

  public ShootCommand(ShooterSubsystem shooter, double rpm) {
    addRequirements(shooter);

    addCommands(
        new InstantCommand(() -> shooter.setRPM(rpm), shooter),

        new WaitUntilCommand(() -> shooter.atSpeed(rpm, 100)),

        new InstantCommand(() -> shooter.extendServo(), shooter),

        RobotBase.isSimulation() ? new InstantCommand(shooter::simulateShot, shooter) : new InstantCommand(),

        new WaitCommand(0.1),

        new InstantCommand(shooter::zeroServo, shooter));
  }
}
