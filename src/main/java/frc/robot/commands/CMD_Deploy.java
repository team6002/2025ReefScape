package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;

public class CMD_Deploy extends SequentialCommandGroup{
    public CMD_Deploy(SUB_CoralHolder p_coralHolder){
        addCommands(
            new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kIntake))
        );
    }
}
