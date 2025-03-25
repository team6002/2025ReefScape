// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.GlobalVariables;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Drive.*;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.Winch.*;
import frc.robot.subsystems.Algae.*;
import frc.robot.subsystems.Wrist.*;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.GroundPivotIOSparkMax;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Pivot.*;
import frc.robot.subsystems.Questimator.QuestNavIOMeta;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Pose2d currentPose;
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
    ,new QuestNavIOMeta()
  );
  final GlobalVariables m_variables = new GlobalVariables();
  final SUB_Intake m_intake = new SUB_Intake(new IntakeIOSparkMax());
  final SUB_Elevator m_elevator = new SUB_Elevator(new ElevatorIOSparkMax());
  final SUB_Pivot m_pivot = new SUB_Pivot(new PivotIOSparkMax());
  final SUB_Wrist m_wrist = new SUB_Wrist(new WristIOSparkMax());
  final SUB_Winch m_winch = new SUB_Winch(new WinchIOSparkMax());
  final SUB_Algae m_algae = new SUB_Algae(new AlgaeIOSparkMax());
  final SUB_GroundPivot m_groundPivot = new SUB_GroundPivot(new GroundPivotIOSparkMax());
  final SUB_GroundIntake m_groundIntake = new SUB_GroundIntake(new GroundIntakeIOSparkMax());
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // m_drivetrain.setDefaultCommand(new CMD_Drive(m_drivetrain, m_driverController));
    m_drivetrain.setDefaultCommand(new CMD_Drive
    (m_drivetrain, m_driverController));
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
    m_driverController.a().onTrue(new InstantCommand(()-> m_winch.setReference(WinchConstants.kHome)));
    m_driverController.y().onFalse(new InstantCommand(()-> m_winch.setReference(WinchConstants.kReadyClimb)));
    
    m_driverController.povUp().onTrue(new InstantCommand(()-> m_drivetrain.zeroHeading()));
    m_driverController.povRight().onTrue(new CMD_Home(m_elevator, m_intake, m_wrist, m_pivot, m_algae, m_variables));

    //operator
    m_operatorController.rightBumper().onTrue(new CMD_Score(m_elevator, m_wrist, m_pivot, m_intake, m_variables));

    m_operatorController.povUp().onTrue(new CMD_ChangeLevel(m_pivot, m_elevator, m_wrist, m_variables, 4));
    m_operatorController.povRight().onTrue(new CMD_ChangeLevel(m_pivot, m_elevator, m_wrist, m_variables, 3));
    m_operatorController.povDown().onTrue(new CMD_ChangeLevel(m_pivot, m_elevator, m_wrist, m_variables, 2));
  }
    
}
