package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeLevelThree extends SequentialCommandGroup{
    public CMD_AlgaeLevelThree(SUB_CoralHolder p_intake, SUB_Wrist p_wrist, SUB_Algae p_algae,
        SUB_Elevator p_elevator, GlobalVariables p_variables, SUB_Pivot p_pivot){
        addRequirements(p_algae);
        addCommands(
            new CMD_ReadyToIntakeAlgaeThree(p_wrist, p_pivot, p_elevator, p_algae, p_intake, p_variables)
            ,new CMD_AlgaeTrigger(p_algae)
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kHolding))
            ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgael3))
            ,new CMD_PivotInPosition(p_pivot)
        );
    }
}
