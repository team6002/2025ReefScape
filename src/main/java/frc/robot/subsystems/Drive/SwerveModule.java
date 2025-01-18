// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // private String m_moduleChannel;//module channel is for telemetry 
   
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private double m_previousVelocity;

  private final SimpleMotorFeedforward m_drivingFeedForward;
  
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
    
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    Logger.recordOutput("speed" + Integer.toString(index), inputs.driveVelocityRadPerSec);
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
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(inputs.turnAbsoluteRadians));    
    // double AccelerationThingy =  (optimizedDesiredState.speedMetersPerSecond - m_previousVelocity)* ModuleConstants.kPAcceleration;
    
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_drivingPIDController.setReference((optimizedDesiredState.speedMetersPerSecond + AccelerationThingy), CANSparkMax.ControlType.kVelocity);
    // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    io.setDriveReference((correctedDesiredState.speedMetersPerSecond), SparkMax.ControlType.kVelocity, 0 ,m_drivingFeedForward.calculate(correctedDesiredState.speedMetersPerSecond));
    io.setTurnReference(correctedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
    m_previousVelocity = m_desiredState.speedMetersPerSecond;
  }
  // public void logging(){
  //   io.updateInputs(inputs);
  //   Logger.processInputs("Drive/Module" + m_moduleChannel, inputs);
    
  // } 
}

