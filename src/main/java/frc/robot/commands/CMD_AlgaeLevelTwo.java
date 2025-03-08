package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeLevelTwo extends SequentialCommandGroup{
    public CMD_AlgaeLevelTwo(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae,
            SUB_CoralHolder p_coralIntake, GlobalVariables p_variables) {
        addCommands(
            new CMD_ReadyToIntakeAlgaeTwo(p_wrist, p_pivot, p_elevator, p_algae)
            ,new ConditionalCommand(
                new PrintCommand("I still hate ur code"),
                new SequentialCommandGroup(
                    new CMD_AlgaeTrigger(p_algae)
                    ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kHolding))
                    ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
                    ,new ConditionalCommand(
                        new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgael3))
                        ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgae))
                        ,()-> p_variables.getAlgaeTarget() == AlgaeTarget.LEVEL_3
                    )
                    ,new CMD_PivotInPosition(p_pivot)
                ),
                ()-> GlobalVariables.m_algaeExceptionMode
            )
        );
    }
}
