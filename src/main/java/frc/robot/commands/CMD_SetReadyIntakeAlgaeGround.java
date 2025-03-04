package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyIntakeAlgaeGround extends SequentialCommandGroup{
    
    public CMD_SetReadyIntakeAlgaeGround(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Algae p_algae,
        GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setAlgaeTarget(AlgaeTarget.GROUND))
            ,new CMD_AlgaeIntakeGround(p_wrist, p_pivot, p_elevator, p_algae)
            ,new CMD_AlgaeTrigger(p_algae)    
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kHolding))
            ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgael3))
            ,new CMD_PivotInPosition(p_pivot)
        );
    }
}
