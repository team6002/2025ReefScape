// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Drive;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Configs;
/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {
 // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = ModuleConstants.kDrivingMotorReduction;
  private static final double TURN_GEAR_RATIO = 46.42;

  private final SparkMax driveSparkFlex;
  private final SparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;
  // private final AnalogInput turnAbsoluteEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;
  
  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  private SparkBaseConfig m_turnConfig = Configs.MAXSwerveModule.turningConfig;
  private SparkBaseConfig m_driveConfig = Configs.MAXSwerveModule.drivingConfig;

  private int m_index;
  public ModuleIOSparkFlex(int index) {
    m_index = index;

    switch (index) {
      case 0:
        driveSparkFlex = new SparkMax(HardwareConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(HardwareConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();
        absoluteEncoderOffset = new Rotation2d(4.75); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkFlex = new SparkMax(HardwareConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(HardwareConstants.kFrontRightTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();
        absoluteEncoderOffset = new Rotation2d(0.183); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkFlex = new SparkMax(HardwareConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(HardwareConstants.kRearLeftTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();
        absoluteEncoderOffset = new Rotation2d(3.183); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkFlex = new SparkMax(HardwareConstants.kRearRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(HardwareConstants.kRearRightTurningCanId, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();
        absoluteEncoderOffset = new Rotation2d(3.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    m_drivingPIDController = driveSparkFlex.getClosedLoopController();
    m_turningPIDController = turnSparkMax.getClosedLoopController();
    
    driveSparkFlex.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkMax.configure(m_turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveSparkFlex.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);
  
    driveEncoder = driveSparkFlex.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    driveEncoder.setPosition(0.0);
    turnRelativeEncoder.setPosition(0.0);

    driveSparkFlex.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec =
        driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSparkFlex.getAppliedOutput() * driveSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkFlex.getOutputCurrent()};


    inputs.turnAbsoluteRadians = turnAbsoluteEncoder.getPosition();

    inputs.turnAbsolutePosition =
        Rotation2d.fromRadians(turnAbsoluteEncoder.getPosition());
        // new Rotation2d(
        //         turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
        //     .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.VisualAbsolutePosition = 
      new Pose2d(m_index,0, inputs.turnAbsolutePosition);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveReference(double desiredState, ControlType ctrlType, int PIDSlot, double feedForward){
    m_drivingPIDController.setReference((desiredState), ctrlType, ClosedLoopSlot.kSlot0, feedForward);
    // m_drivingPIDController.setReference(desiredState, ctrlType);
  }
  @Override
  public void setTurnReference(double desiredState, ControlType ctrlType){
    m_turningPIDController.setReference(desiredState, ctrlType);
  }
}