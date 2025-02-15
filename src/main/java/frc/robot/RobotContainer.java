// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.GlobalVariables;
import frc.GlobalVariables.IntakeState;
import frc.GlobalVariables.Mode;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Drive.*;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.Winch.SUB_Winch;
import frc.robot.subsystems.Winch.WinchIOSparkMax;
import frc.robot.subsystems.Algae.AlgaeIOSparkMax;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.*;
import frc.robot.subsystems.Wrist.*;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.ElevatorPivot.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // final SUB_Vision m_vision = new SUB_Vision(new VisionIOPhoton());
  final SUB_Vision m_vision = new SUB_Vision(new VisionIOPhoton());
  final SUB_Drivetrain m_drivetrain = new SUB_Drivetrain(
    new GyroIONavX()
    ,new ModuleIOSparkFlex(0)
    ,new ModuleIOSparkFlex(1)
    ,new ModuleIOSparkFlex(2)
    ,new ModuleIOSparkFlex(3)
    ,m_vision
    );
  final GlobalVariables m_variables = new GlobalVariables();
  // final SUB_CoralHolder m_coralIntake = new SUB_CoralHolder(new CoralHolderIOSparkMax());
  // final SUB_Elevator m_elevator = new SUB_Elevator(new ElevatorIOSparkMax());
  // final SUB_ElevatorPivot m_elevatorPivot = new SUB_ElevatorPivot(new ElevatorPivotIOSparkMax());
  // final SUB_Wrist m_wrist = new SUB_Wrist(new WristIOSparkMax());


  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // m_robotDrive.setDefaultCommand(new CMD_Drive(m_robotDrive, m_driverController));
    m_drivetrain.setDefaultCommand(new CMD_Drive(m_drivetrain, m_driverController).withTimeout(1));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //driver
    
    m_driverController.a().onTrue(new SequentialCommandGroup(
      new CMD_DriveAlignVision(m_drivetrain, m_vision, m_driverController),
      new CMD_DriveAlignVisionAdjust(m_drivetrain, m_vision, m_driverController, Units.inchesToMeters(0), Units.inchesToMeters(7), 0) 
      ));
  }
}
