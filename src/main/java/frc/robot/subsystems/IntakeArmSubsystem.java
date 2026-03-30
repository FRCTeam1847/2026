// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {
/*
 * my foobar stuff was riffing on some example code at 
 * 
 * https://github.com/REVrobotics/REVLib-Examples/blob/814041d57204e66ca93cdf15a5c77b44e0123bd1/Java/SPARK/Open%20Loop%20Arcade%20Drive/src/main/java/frc/robot/Robot.java#L60

  got there from a post on Chief Delphi:
  https://www.chiefdelphi.com/t/how-to-set-motors-into-follower-mode-in-2025/485709/2
 
 
  */

    private final SparkMax motor;
    private final DutyCycleEncoder absoluteEncoder;
    private final PIDController pid;
  private static final double GEAR_RATIO = 58.3;
    public IntakeArmSubsystem() {

        // Motor
        motor = new SparkMax(IntakeArmConstants.kMotorID, MotorType.kBrushless);
       // followMotor = new SparkMax(IntakeArmConstants.kfollowMotorID, MotorType.kBrushless);
        

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig configs = new SparkMaxConfig();       
       // SparkMaxConfig followConfigs = new SparkMaxConfig();     
        
        /* *******************************************************************************************
         * - there's some additional stuff in the REV sample code about creating a global config 
         * and using the `.apply` method so that all the configs involved in the lead/follow 
         * arrangement have a common base
         *
         * - for now, I'm just treating that as assumed   ***BUT THAT ABSOLUTELY MUST BE VERIFIED!!!!***
         * 
         * - also, this is code from 2025, so be sure there aren't changes for 2026 that haven't 
         * been accounted for
         ******************************************************************************************* */

        globalConfig
            .idleMode(IdleMode.kCoast);

       // followConfigs
          //  .apply(globalConfig)     // - this is for illustration only; showing what the application  
                                    //   of a "global config" would look like
                                    // - pretty sure this right here be hinky AF in any sort of real application 
            // ***CRITICAL*** - since the motors are facing each other, one of them MUST be reversed so they're not fighting each other
            
            // and here's the follow, finally
           // .follow(IntakeArmConstants.kMotorID, true);
        
       // followMotor.configure(followConfigs,
              // ResetMode.kResetSafeParameters,
              // PersistMode.kPersistParameters);

        configs.apply(globalConfig);
        configs.absoluteEncoder
        .positionConversionFactor(360.0 / GEAR_RATIO)
        .velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);

        motor.configure(configs,
               ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters);

        // Absolute encoder on DIO
        absoluteEncoder = new DutyCycleEncoder(IntakeArmConstants.kEncoderDIO);

        // WPILib PID (since encoder is on RIO)
        pid = new PIDController(
                IntakeArmConstants.kP,
                IntakeArmConstants.kI,
                IntakeArmConstants.kD
        );

        pid.setTolerance(0.01);
        // Enable continuous input (important for absolute encoders)
       pid.enableContinuousInput(0.0, 1.0);
    }

    // Get absolute position (0.0 - 1.0 rotations)
    public double getAbsolutePosition() {
        double position = absoluteEncoder.get();

        position -= IntakeArmConstants.kEncoderOffset;

        // Wrap between 0 and 1
        return MathUtil.inputModulus(position, 0.0, 1.0);
    }

    // Convert to degrees (easier for students)
    public double getDegrees() {
        return getAbsolutePosition() * 360.0;
    }

    // Open loop
    public Command run(double speed) {
      return  run(() -> {
        motor.set(speed);
    });

    }

    public void stop() {
        motor.stopMotor();
    }

    // Closed-loop position control (degrees)
    public void moveToAngle(double targetDegrees) {

        double targetRotations = targetDegrees / 360.0;

        double output = pid.calculate(getAbsolutePosition(), targetRotations);

        output = MathUtil.clamp(output, -0.5, 0.5);  // start small
        
        motor.set(output);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        // This method will be called once per scheduler run during simulation
 SmartDashboard.putNumber("Current Position", getDegrees());
 SmartDashboard.putNumber("Raw Encoder", absoluteEncoder.get());

    Logger.recordOutput("INTAKEARM/Current Position", getDegrees());
    Logger.recordOutput("INTAKEARM/Target Position", 90);
   // Logger.recordOutput()
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
 SmartDashboard.putNumber("Current Position", absoluteEncoder.get());
    Logger.recordOutput("INTAKEARM/Current Position", absoluteEncoder.get());
   // Logger.recordOutput()
  }
}