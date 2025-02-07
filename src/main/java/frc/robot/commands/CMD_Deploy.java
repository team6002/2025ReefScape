package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Deploy extends SequentialCommandGroup{
    public CMD_Deploy(SUB_CoralHolder p_coralHolder, SUB_Wrist p_wrist, SUB_ElevatorPivot p_pivot){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(p_wrist.getPosition() - Math.toRadians(30)))
            ,new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kReverse))
        );
    }
}
