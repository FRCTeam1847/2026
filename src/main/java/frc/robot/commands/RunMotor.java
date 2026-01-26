// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunMotor extends Command {
  private final MotorSubsystem motor;
  private final double percent;

  public RunMotor(MotorSubsystem motor, double percent) {
        this.motor = motor;
        this.percent = percent;
        addRequirements(motor);
    }

    @Override
    public void execute() {
        motor.run(percent);
    }

    @Override
    public void end(boolean interrupted) {
        motor.stop();
    }
}
