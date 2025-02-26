// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autos.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<SequentialCommandGroup> m_autonomousChooser = new SendableChooser<SequentialCommandGroup>();

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    }
    
    Logger.registerURCL(URCL.startExternal());
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_autonomousChooser.addOption("AUTO_BlueLeft1", new AUTO_BlueLeft1(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_SideTuning", new AUTO_SideTuning(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_AccelTuning", new AUTO_AccelTuning(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_AccelTuningF", new AUTO_AccelTuningF(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_AccelTuningB", new AUTO_AccelTuningB(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_WheelTuningF", new AUTO_WheelTuningForwards(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_WheelTuningB", new AUTO_WheelTuningBackwards(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_WheelTuning", new AUTO_WheelTuning(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_Tuning", new AUTO_Tuning(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_TuningForwards", new AUTO_TuningForwards(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_TuningBackwards", new AUTO_TuningBackwards(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_BlueLeft", new AUTO_BlueLeft(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake));
    m_autonomousChooser.addOption("AUTO_BlueRight", new AUTO_BlueRight(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake));
    m_autonomousChooser.addOption("AUTO_BlueLeft244", new AUTO_BlueLeft244(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    m_autonomousChooser.addOption("AUTO_BlueRight244", new AUTO_BlueRight244(m_robotContainer.m_drivetrain, m_robotContainer.m_pivot, m_robotContainer.m_wrist, m_robotContainer.m_elevator, m_robotContainer.m_coralIntake, m_robotContainer.m_algae));
    SmartDashboard.putData(m_autonomousChooser);

    m_robotContainer.m_pivot.reset();
    m_robotContainer.m_wrist.reset();
    m_robotContainer.m_elevator.resetEncoder();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_pivot.reset();
    m_robotContainer.m_wrist.reset();
    m_robotContainer.m_elevator.resetEncoder();

    m_autonomousCommand = m_autonomousChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.m_pivot.reset();
    m_robotContainer.m_wrist.reset();
    m_robotContainer.m_elevator.resetEncoder();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}