// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_AlgaeDeploy extends SequentialCommandGroup {
  /** Creates a new CMD_AlgaeDeploy. */
 
  public CMD_AlgaeDeploy(SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(p_intake);
    addCommands(
          new InstantCommand(()->p_intake.setVoltage(-12))
          // m_variables.setRobotState(RobotState.READY);
          ,new WaitCommand(1).andThen(new InstantCommand(()-> GlobalVariables.m_haveAlgae = false))
          ,new InstantCommand(()->p_flippyWrist.setGoal(FlippyWristConstants.kIntakeGround))
                
    );
  }
}
