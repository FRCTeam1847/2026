// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.net.PortForwarder;
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
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.Mode;

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
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() -> drivebase.getPose());
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  // // private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final IntakeArmSubsystem intakeArmSubsystem = new
  // IntakeArmSubsystem();

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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerNamedCommands();
    // Configure the trigger bindings

    drivebase.setupPathPlanner();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    // NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // NamedCommands.registerCommand(null, getAutonomousCommand());

    // // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    // // // Set the default auto (do nothing)
    // autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // // Add a simple auto option to have the robot drive forward for 1 second then
    // // stop
    // autoChooser.addOption("Drive Forward",
    // drivebase.driveForward().withTimeout(1));
    // autoChooser.addOption("Drive Forward",
    // drivebase.driveForward().withTimeout(1));

    // // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    turretSubsystem.setSuppliers(
        drivebase::getPose, // Pose2d supplier
        drivebase::getFieldVelocity // ChassisSpeeds supplier
    );

  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("shoot",
        new ShootCommand(shooterSubsystem, indexerSubsystem)
            .alongWith(intakeSubsystem.oscillateRollersCommand(3, 0.15)));
    NamedCommands.registerCommand("intakeUp",
        intakeSubsystem.moveToAngleCommand(IntakeSubsystem.MIN_ANGLE + 5));
    NamedCommands.registerCommand("intakeDown",
        intakeSubsystem.moveToAngleCommand(IntakeSubsystem.MAX_ANGLE - 5));
    NamedCommands.registerCommand("intakeFuel", intakeSubsystem.intakeFuel());
    NamedCommands.registerCommand("outputFuel", intakeSubsystem.throwFuel());
    NamedCommands.registerCommand("intakeFuelStop", intakeSubsystem.stopIntakeCommand());
    NamedCommands.registerCommand("indexerForward",
        indexerSubsystem.runIndexer(Constants.IndexerConstants.INDEXER_SPEED));
    NamedCommands.registerCommand("indexerBackward",
        indexerSubsystem.runIndexer(-Constants.IndexerConstants.INDEXER_SPEED));
    NamedCommands.registerCommand("indexerStop", indexerSubsystem.stop());
    NamedCommands.registerCommand("setManual", turretSubsystem.setManual());
    NamedCommands.registerCommand("setTracking", turretSubsystem.setTracking());

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
    driverController.circle().onTrue(NamedCommands.getCommand("intakeUp"));
    driverController.cross().onTrue((NamedCommands.getCommand("intakeDown")));
    driverController.R1().whileTrue(NamedCommands.getCommand("intakeFuel"))
        .whileFalse(NamedCommands.getCommand("intakeFuelStop"));
    driverController.R2().whileTrue(NamedCommands.getCommand("outputFuel"))
        .whileFalse(NamedCommands.getCommand("intakeFuelStop"));
    // #endregion

    // #region Indexer Manual Controls
    driverController.povUp().whileTrue(NamedCommands.getCommand("indexerForward"))
        .whileFalse(NamedCommands.getCommand("indexerStop"));
    driverController.povDown().whileTrue(NamedCommands.getCommand("indexerBackward"))
        .whileFalse(NamedCommands.getCommand("indexerStop"));
    // #endregion

    // #region Turret Controls
    // turretSubsystem.setDefaultCommand(turretSubsystem.run(() ->
    // turretSubsystem.setPercent(0.0)));
    // driverController.L2().onTrue(turretSubsystem.trackAprilTagCommand());
    // Commands.runOnce(() -> {
    // if (aimAtHubCommand.isScheduled()) {
    // aimAtHubCommand.cancel(); // stop aiming
    // } else {
    // aimAtHubCommand.schedule(); // start aiming
    // }
    // });

    // driverController.povRight().onTrue(turretSubsystem.increaseAngleCommand(true));
    // driverController.povLeft().onTrue(turretSubsystem.increaseAngleCommand(false));
    driverController.povRight().whileTrue(
        Commands.runOnce(() -> turretSubsystem.adjustAimOffset(3.0), turretSubsystem));

    driverController.povLeft().whileTrue(
        Commands.runOnce(() -> turretSubsystem.adjustAimOffset(-3.0), turretSubsystem));
    driverController.R3().onTrue(
        Commands.runOnce(() -> turretSubsystem.resetAimOffset(), turretSubsystem));

    // Hold A to auto-aim turret (AUTO_AIM mode)
    // driverController.cross().whileTrue(
    // turretSubsystem.setModeCommand(TurretSubsystem.TurretMode.AUTO_AIM)).onFalse(
    // turretSubsystem.setModeCommand(TurretSubsystem.TurretMode.HOLD_ANGLE));

    // // Manual turret overrides for debugging
    // driverController.L2().whileTrue(
    // turretSubsystem.run(() -> turretSubsystem.setManualPercent(0.2)));
    // driverController.R2().whileTrue(
    // turretSubsystem.run(() -> turretSubsystem.setManualPercent(-0.2)));

    // // Example: field-lock mode (hold hub)
    driverController.L2().onTrue(turretSubsystem.toggleManualHubTrackingCommand());

    // // Example: scan mode for testing
    // driverController.circle().whileTrue(
    // turretSubsystem.setModeCommand(TurretSubsystem.TurretMode.SCAN)).onFalse(
    // turretSubsystem.setModeCommand(TurretSubsystem.TurretMode.HOLD_ANGLE));
    // #endregion

    // #region Shooter Controls
    // shooterSubsystem
    // .setDefaultCommand(shooterSubsystem.dynamicHoodCommand(() ->
    // shooterSubsystem.calculateLaunchAngle()));
    // driverController.triangle().whileTrue(shooterSubsystem.raiseServo());
    // driverController.square().whileTrue(shooterSubsystem.lowerServo());
    // driverController.R2().toggleOnTrue(new InstantCommand(() ->
    // shooterSubsystem.toggleDynamicHood()));

    driverController.L1()
        .whileTrue(NamedCommands.getCommand("shoot"))
    // .onFalse(new InstantCommand(() -> {
    // shooterSubsystem.stop();
    // indexerSubsystem.stop();
    // }))
    ;
    // driverController.L1().whileTrue(new InstantCommand(() -> {
    // double rpm = shooterSubsystem.calculateFlywheelRPM();
    // shooterSubsystem.setRPM(rpm);
    // })).whileFalse(new InstantCommand(() -> shooterSubsystem.stop()));
    // // #endregion

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
    // drivebase.setMotorBrake(brake);
  }
}