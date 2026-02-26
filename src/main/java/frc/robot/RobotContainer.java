// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverController = new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(drivebase::getPose, turretSubsystem);
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1)
      .withControllerRotationAxis(driverController::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.5)
      .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX())
      .withControllerRotationAxis(() -> driverController.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.5)
      .allianceRelativeControl(true);

  private final Command aimAtHubCommand = new RunCommand(
      () -> turretSubsystem.aimAtHub(drivebase.getPose()),
      turretSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // // Add a simple auto option to have the robot drive forward for 1 second then
    // // stop
    // autoChooser.addOption("Drive Forward",
    // drivebase.driveForward().withTimeout(1));
    // autoChooser.addOption("Drive Forward",
    // drivebase.driveForward().withTimeout(1));

    // // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    turretSubsystem.zeroToAbsolute();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // #region Drive Controls
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    // #endregion

    // #region Intake Controls
    driverController.circle().whileTrue(intakeSubsystem.moveToAngle(90));// .onFalse(intakeSubsystem.stopArm());
    driverController.cross().whileTrue(intakeSubsystem.moveToAngle(0));
    // #endregion

    // #region Indexer Manual Controls
    driverController.povUp().whileTrue(indexerSubsystem.runIndexer(0.5));
    driverController.povDown().whileTrue(indexerSubsystem.runIndexer(-0.5));
    // #endregion

    // #region Turret Controls
    turretSubsystem.setDefaultCommand(turretSubsystem.run(() -> turretSubsystem.setPercent(0.0)));
    driverController.L2().onTrue(
        Commands.runOnce(() -> {
          if (aimAtHubCommand.isScheduled()) {
            aimAtHubCommand.cancel(); // stop aiming
          } else {
            aimAtHubCommand.schedule(); // start aiming
          }
        }));

    driverController.povRight().onTrue(turretSubsystem.setPercentageCommand(0.25));
    driverController.povLeft().onTrue(turretSubsystem.setPercentageCommand(-0.25));
    // #endregion

    // #region Shooter Controls
    shooterSubsystem
        .setDefaultCommand(shooterSubsystem.dynamicHoodCommand(() -> shooterSubsystem.calculateLaunchAngle()));
    driverController.triangle().whileTrue(shooterSubsystem.raiseServo());
    driverController.square().whileTrue(shooterSubsystem.lowerServo());
    driverController.R2().toggleOnTrue(new InstantCommand(() -> shooterSubsystem.toggleDynamicHood()));

    driverController.L1()
        .whileTrue(new ShootCommand(shooterSubsystem, indexerSubsystem)).onFalse(new InstantCommand(() -> {
          shooterSubsystem.stop();
          indexerSubsystem.stop();
        }));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}