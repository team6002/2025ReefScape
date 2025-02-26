package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_DeployLevelThree extends SequentialCommandGroup{
    public CMD_DeployLevelThree(SUB_CoralHolder p_intake, SUB_Wrist p_wrist){
        addCommands(
            new InstantCommand(()-> p_intake.setVoltage(CoralHolderConstants.kReverse))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kStowing))
        );
    }
}
