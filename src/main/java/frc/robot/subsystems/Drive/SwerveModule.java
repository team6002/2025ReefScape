// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private double prevVelo;

  // private String m_moduleChannel;//module channel is for telemetry 
   
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private final SimpleMotorFeedforward m_drivingFeedForward;
  private final SimpleMotorFeedforward m_autoFeedForward;
  
  private double m_chassisAngularOffset = 0;
  // private final ModuleIO io;
  // private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  /** 
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(ModuleIO io, int index, double chassisAngularOffset) {   
    this.io = io;
    this.index = index;

    m_chassisAngularOffset = chassisAngularOffset;
    // m_moduleChannel = p_moduleChannel;

    m_drivingFeedForward = new SimpleMotorFeedforward(ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA);
    m_autoFeedForward = new SimpleMotorFeedforward(ModuleConstants.kAutoS, ModuleConstants.kAutoV, ModuleConstants.kAutoA);
    
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    Logger.recordOutput("speed" + Integer.toString(index), Math.abs(inputs.driveVelocityRadPerSec));
    Logger.recordOutput("desiredSpeed" + Integer.toString(index), m_desiredState.speedMetersPerSecond);
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(inputs.driveVelocityRadPerSec,
        new Rotation2d(inputs.turnAbsoluteRadians - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        inputs.drivePositionRad,
        new Rotation2d(inputs.turnAbsoluteRadians - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

  public void setDesiredStateAuto(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    
    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(inputs.turnAbsoluteRadians));    
    // double AccelerationThingy =  (optimizedDesiredState.speedMetersPerSecond - m_previousVelocity)* ModuleConstants.kPAcceleration;
    // prevVelo = io.getVelo
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_drivingPIDController.setReference((optimizedDesiredState.speedMetersPerSecond + AccelerationThingy), CANSparkMax.ControlType.kVelocity);
    // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    // Logger.recordOutput("DrivePrevVelo" + index, (prevVelo));
    // Logger.recordOutput("DriveAccel" + index, ((correctedDesiredState.speedMetersPerSecond - prevVelo)/ .02));
    
    double acceleration = ((correctedDesiredState.speedMetersPerSecond - prevVelo)/0.02);
    double feedForward = m_autoFeedForward.calculate(correctedDesiredState.speedMetersPerSecond, acceleration);
    // iwwww
    // double feedForward = ModuleConstants.kAutoS 
    io.setDriveReference((correctedDesiredState.speedMetersPerSecond), SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1 , feedForward);
    io.setTurnReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
    Logger.recordOutput("DriveFFOutput" + index, feedForward);
    
    m_desiredState = desiredState;
    prevVelo = correctedDesiredState.speedMetersPerSecond;
  
  }
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    
    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(inputs.turnAbsoluteRadians));    
    // double AccelerationThingy =  (optimizedDesiredState.speedMetersPerSecond - m_previousVelocity)* ModuleConstants.kPAcceleration;
    // prevVelo = io.getVelo
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_drivingPIDController.setReference((optimizedDesiredState.speedMetersPerSecond + AccelerationThingy), CANSparkMax.ControlType.kVelocity);
    // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    // Logger.recordOutput("DriveFFOutput" + index, m_drivingFeedForward.calculate(correctedDesiredState.speedMetersPerSecond, (correctedDesiredState.speedMetersPerSecond - prevVelo)/ .02));
    // Logger.recordOutput("DrivePrevVelo" + index, (prevVelo));
    // Logger.recordOutput("DriveAccel" + index, ((correctedDesiredState.speedMetersPerSecond - prevVelo)/ .02));
    io.setDriveReference((correctedDesiredState.speedMetersPerSecond), SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0 ,m_drivingFeedForward.calculate(correctedDesiredState.speedMetersPerSecond));
    io.setTurnReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
    prevVelo = correctedDesiredState.speedMetersPerSecond;
  
  }

}

