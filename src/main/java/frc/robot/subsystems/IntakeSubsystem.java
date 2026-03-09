package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  // ========================
  // Hardware
  // ========================
  private final SparkMax armMotor;
  private final DutyCycleEncoder intakeAbsoluteEncoder;

  // Roller (Kraken X60)
  private final TalonFX rollerMotor;
  private final DutyCycleOut rollerRequest;

  // ========================
  // Constants
  // ========================
  private static final double GEAR_RATIO = 160.0;
  private static final double ARM_LENGTH = 0.4; // meters
  private static final double ARM_MASS = 3.17515; // kg
  private static final double MIN_ANGLE = 0.0;
  private static final double MAX_ANGLE = Math.PI / 2.0;

  // Feedforward values
  private static final double kS = 0.0;
  private static final double kG = 0.12;
  private static final double kV = 3.10;
  private static final double kA = 0.0;

  public IntakeSubsystem() {
    rollerMotor = new TalonFX(IntakeConstants.ROLLER_Kraken_ID);
    rollerRequest = new DutyCycleOut(0);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    armMotor = new SparkMax(IntakeConstants.ARM_Neo_ID, MotorType.kBrushless);
    intakeAbsoluteEncoder = new DutyCycleEncoder(1);
    pid.setTolerance(Math.toRadians(2));
    SmartDashboard.putData("Intake Arm", mech);
  }

  // ========================
  // Control
  // ========================

  private final PIDController pid = new PIDController(0.01, 0.0, 0);
  private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  private double targetAngleRad = Math.toRadians(45);

  // Store commanded voltage manually
  private double appliedVoltage = 0.0;

  // ========================
  // Simulation
  // ========================

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
      GEAR_RATIO,
      SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS),
      ARM_LENGTH,
      MIN_ANGLE,
      MAX_ANGLE,
      true,
      Math.toRadians(45));

  private final Mechanism2d mech = new Mechanism2d(3, 3);
  private final MechanismRoot2d root = mech.getRoot("root", 1.5, 1.5);
  private final MechanismLigament2d armLigament = root.append(new MechanismLigament2d("arm", ARM_LENGTH, 45));

  // ========================
  // Angle Getter
  // ========================

  public double getAngleRadians() {
    if (RobotBase.isSimulation()) {
      return armSim.getAngleRads();
    } else {
      return intakeAbsoluteEncoder.get() * 2.0 * Math.PI;
    }
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public void setTargetDegrees(double degrees) {
    targetAngleRad = Math.toRadians(degrees);
  }

  // ---------------- ROLLERS ----------------

  public void runRollers(double percent) {
    rollerMotor.setControl(
        rollerRequest.withOutput(percent));
  }

  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  // ========================
  // Periodic
  // ========================

  @Override
  public void periodic() {

    double currentAngle = getAngleRadians();

    double pidOutput = pid.calculate(currentAngle, targetAngleRad);
    double ffOutput = ff.calculate(targetAngleRad, 0);

    appliedVoltage = MathUtil.clamp(pidOutput + ffOutput, -12, 12);

    armMotor.setVoltage(appliedVoltage);

    Logger.recordOutput("IntakeArm/Arm Angle Deg", Math.toDegrees(currentAngle));
    Logger.recordOutput("IntakeArm/Target Deg", Math.toDegrees(targetAngleRad));
    Logger.recordOutput("IntakeArm/Applied Voltage", appliedVoltage);
    Logger.recordOutput("IntakeArm/Rollers/RPM", rollerMotor.getVelocity().getValueAsDouble() * 60.0);
    Logger.recordOutput("IntakeArm/Rollers/Percentage", rollerRequest.Output);
  }

  @Override
  public void simulationPeriodic() {

    // Feed the SIM the stored voltage
    armSim.setInput(appliedVoltage);
    armSim.update(0.02);

    armLigament.setAngle(Math.toDegrees(armSim.getAngleRads()));
  }

  // ========================
  // Commands
  // ========================

  public Command moveToAngle(double degrees) {
    return run(() -> setTargetDegrees(degrees))
        .until(this::atSetpoint);
  }

  public Command intakeFuel(){
    return run(()->runRollers(-0.4));
  }
  public Command stopIntakeCommand(){
    return run(()-> rollerMotor.stopMotor());
  }
  public Command stopArm() {
    return runOnce(() -> {
      appliedVoltage = 0;
      armMotor.setVoltage(0);
    });
  }
}
