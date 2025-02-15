package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_DeployLevelFour extends SequentialCommandGroup{
    public CMD_DeployLevelFour(SUB_CoralHolder p_coralHolder, SUB_Wrist p_wrist){
        addCommands(
            new ParallelCommandGroup(
                new CMD_IntakeJackhammer(p_coralHolder).withTimeout(.8)
                ,new SequentialCommandGroup(
                    new WaitCommand(.075)
                    ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kStowing))   
                )
            )
        );
    }
}
