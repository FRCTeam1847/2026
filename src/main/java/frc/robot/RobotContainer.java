// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommandV2;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.simulation.FieldSimulation;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.V2.ShooterSubsystem;
import static edu.wpi.first.units.Units.Degrees;
import swervelib.SwerveInputStream;
import java.io.File;

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
    final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    // private final TurretSubsystemV2 turretSubsystem = new TurretSubsystemV2();
    // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() ->
    // drivebase.getPose());
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() -> drivebase.getPose());
    private final ShootOnTheMoveCommand shootOnTheMoveCommand = new ShootOnTheMoveCommand(shooterSubsystem, drivebase,
            () -> drivebase.getPose(),
            () -> drivebase.getFieldVelocity(),
            new Pose2d(shooterSubsystem.getHub().getX(),
                    shooterSubsystem.getHub().getY(), new Rotation2d()));

    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    private final ShootCommandV2 shootCommand = new ShootCommandV2(shooterSubsystem, indexerSubsystem, drivebase);

    // // private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // private final IntakeArmSubsystem intakeArmSubsystem = new
    // IntakeArmSubsystem();

    // Establish a Sendable Chooser that will be able to be sent to the
    // SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;

    private final FieldSimulation fieldSimulation;

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
            .withControllerRotationAxis(() -> driverController.getRightX())
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
        // Have the autoChooser pull in all PathPlanner autos as options
        autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();

        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        if (RobotBase.isSimulation()) {
            fieldSimulation = new FieldSimulation(drivebase.getSwerveDrive().getMapleSimDrive().get());
        } else {
            fieldSimulation = null;
        }

    }

    private void registerNamedCommands() {
        // #region Shooter Commands
        // Shooter part
        // Run both shoot on the move and shoot command
        NamedCommands.registerCommand("shootOnTheMove",
                new ParallelCommandGroup(shootOnTheMoveCommand, shootCommand));
        // NamedCommands.registerCommand("shootManual",
        // new
        // ParallelCommandGroup(shooterSubsystem.getFlywheel().setRPMCommand(RPM.of(1000)),
        // shootCommand));
        // Hood Part
        NamedCommands.registerCommand("hoodUp", shooterSubsystem.getHood().setAngle(Degrees.of(25)));
        NamedCommands.registerCommand("hoodDown", shooterSubsystem.getHood().setAngle(Degrees.of(56)));

        // Turret Part
        NamedCommands.registerCommand("Turret0", shooterSubsystem.getTurret().setTurrentAngleCommand(Degrees.of(0)));
        NamedCommands.registerCommand("Turret90", shooterSubsystem.getTurret().setTurrentAngleCommand(Degrees.of(90)));
        NamedCommands.registerCommand("TurretNeg90",
                shooterSubsystem.getTurret().setTurrentAngleCommand(Degrees.of(-90)));
        NamedCommands.registerCommand("TurretNeg180",
                shooterSubsystem.getTurret().setTurrentAngleCommand(Degrees.of(-180)));

        // #endregion

        // #region Intake Named Commands
        NamedCommands.registerCommand("intakeUp",
                intakeSubsystem.moveToAngleCommand(IntakeSubsystem.MIN_ANGLE + 5));
        NamedCommands.registerCommand("intakeDown",
                intakeSubsystem.moveToAngleCommand(IntakeSubsystem.MAX_ANGLE - 5));
        // #endregion
        // #region Rollers Commands
        NamedCommands.registerCommand("collectFuel", intakeSubsystem.collectFuel());
        NamedCommands.registerCommand("outputFuel", intakeSubsystem.dropFuel());
        // #endregion

        // #region Indexer Commands
        NamedCommands.registerCommand("indexForward", indexerSubsystem.IndexForward());
        NamedCommands.registerCommand("indexReverse", indexerSubsystem.IndexReverse());
        NamedCommands.registerCommand("indexManual", shootCommand);

        // #endregion
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

        // #region Shooter
        // Should only use 1000 RPM if we are in test mode. If not use manual shoot
        RobotModeTriggers.teleop().and(driverController.rightTrigger())
                .whileTrue(NamedCommands.getCommand("shootOnTheMove"));
        // RobotModeTriggers.test().and(driverController.rightTrigger())
        //         .whileTrue(NamedCommands.getCommand("shootManual"));

        driverController.y().whileTrue(NamedCommands.getCommand("hoodUp"));
        driverController.x().whileTrue(NamedCommands.getCommand("hoodDown"));
        // #endregion

        // #region Intake
        driverController.leftTrigger().whileTrue(NamedCommands.getCommand("collectFuel"));
        driverController.a().whileTrue(NamedCommands.getCommand("intakeDown"));
        driverController.b().whileTrue(NamedCommands.getCommand("intakeUp"));
        // #endregion

        // #region Turret
        driverController.povUp().whileTrue(NamedCommands.getCommand("turret0"));
        driverController.povDown().whileTrue(NamedCommands.getCommand("turretNeg180"));
        driverController.povRight().whileTrue(NamedCommands.getCommand("turret90"));
        driverController.povLeft().whileTrue(NamedCommands.getCommand("turretNeg90"));
        // #endregion

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

    public void simulationPeriodic() {
        if (fieldSimulation != null) {
            fieldSimulation.periodic();
        }
    }
}