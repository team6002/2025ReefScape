package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyIntakeAlgaeGround extends SequentialCommandGroup{
    
    public CMD_SetReadyIntakeAlgaeGround(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Algae p_algae,
        SUB_CoralHolder p_intake, GlobalVariables p_variables){
        addCommands(
            new ParallelCommandGroup(
                //normal algae ground sequence
                new SequentialCommandGroup(
                    new InstantCommand(()-> p_variables.setAlgaeTarget(AlgaeTarget.GROUND))
                    ,new CMD_AlgaeIntakeGround(p_wrist, p_pivot, p_elevator, p_algae)
                    ,new CMD_AlgaeTrigger(p_algae)    
                    ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kHolding))
                    ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
                    ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgael3))
                    ,new CMD_PivotInPosition(p_pivot)
                )
                //check if have coral
                ,new SequentialCommandGroup(
                    new CMD_CheckCoral(p_intake)
                    ,new ConditionalCommand(
                        new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_TO_INTAKE))
                        ,new InstantCommand(()-> p_variables.setRobotState(RobotState.HOME))
                        ,()-> GlobalVariables.m_haveCoral
                    )
                )
            )
            
        );
    }
}
