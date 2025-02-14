// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
  // The robot's subsystems
  // final SUB_Vision m_vision = new SUB_Vision(new VisionIOPhoton());
  // final SUB_Vision m_vision = new SUB_Vision(new VisionIOPhoton());
  final SUB_Drivetrain m_robotDrive = new SUB_Drivetrain(
    new GyroIONavX()
    ,new ModuleIOSparkFlex(0)
    ,new ModuleIOSparkFlex(1)
    ,new ModuleIOSparkFlex(2)
    ,new ModuleIOSparkFlex(3)
    // ,m_vision
    );
  final GlobalVariables m_variables = new GlobalVariables();
  final SUB_CoralHolder m_coralIntake = new SUB_CoralHolder(new CoralHolderIOSparkMax());
  final SUB_Elevator m_elevator = new SUB_Elevator(new ElevatorIOSparkMax());
  final SUB_ElevatorPivot m_elevatorPivot = new SUB_ElevatorPivot(new ElevatorPivotIOSparkMax());
  final SUB_Wrist m_wrist = new SUB_Wrist(new WristIOSparkMax());
  final SUB_Winch m_winch = new SUB_Winch(new WinchIOSparkMax());
  final SUB_Algae m_algae = new SUB_Algae(new AlgaeIOSparkMax());
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
    m_robotDrive.setDefaultCommand(new CMD_Drive(m_robotDrive, m_driverController).withTimeout(1));
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
    m_driverController.rightBumper().onTrue(new CMD_Score(m_elevator, m_wrist, m_coralIntake, m_elevatorPivot, m_variables));
    m_driverController.leftBumper().onTrue(new CMD_Home(m_elevator, m_coralIntake, m_wrist, m_elevatorPivot)
      .andThen(new InstantCommand(()-> m_variables.setRobotState(RobotState.HOME))));
    m_driverController.povLeft().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 1));
    m_driverController.povDown().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 2));
    m_driverController.povRight().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 3));
    m_driverController.povUp().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 4));
    m_driverController.back().onTrue(new ConditionalCommand(
      new InstantCommand(()-> m_variables.setMode(Mode.DEFENSIVE))
      ,new InstantCommand(()-> m_variables.setMode(Mode.OFFENSIVE))
      ,()-> m_variables.isMode(Mode.OFFENSIVE)
    ));
    m_driverController.start().onTrue(new InstantCommand(()-> m_robotDrive.zeroHeading()));
    m_driverController.a().onTrue(new InstantCommand(()->m_algae.setReference(8)));
    m_driverController.b().onTrue(new InstantCommand(()-> m_algae.setReference(2)));
    m_driverController.x().onTrue(new InstantCommand(()-> m_algae.setReference(-5)));
    // m_driverController.x().onTrue(new InstantCommand(()-> m_variables.setIntakeState(IntakeState.ALGAE)));
    // m_driverController.y().onTrue(new InstantCommand(()-> m_variables.setIntakeState(IntakeState.CORAL)));
    // m_driverController.a().onTrue(new CMD_ElevatorReset(m_elevator));
    // m_driverController.b().onTrue(new CMD_AutoDeploy(m_wrist, m_coralIntake, m_variables, m_driverController));

    // m_driverController.start().onTrue(new InstantCommand(()-> m_coralIntake.setReference(-2000)));
    // m_driverController.back().onTrue(new InstantCommand(()-> m_coralIntake.setReference(0)));
    //operator
    // m_operatorController.a().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 2));
    // m_operatorController.b().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 3));
    // m_operatorController.x().onTrue(new InstantCommand(()-> GlobalVariables.m_targetLevel = 4));
  }
}
