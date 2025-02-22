package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_YeetAlgae extends SequentialCommandGroup{
    public CMD_YeetAlgae(SUB_Wrist p_wrist, SUB_Algae p_algae){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(WristConstants.kAlgaeYeet))
            ,new CMD_WristInPosition(p_wrist)
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kReverse))
            ,new WaitCommand(.33)
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kOff))
            ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = false)
            ,new CMD_WristSetDeploy(p_wrist)
        );
    }
}
