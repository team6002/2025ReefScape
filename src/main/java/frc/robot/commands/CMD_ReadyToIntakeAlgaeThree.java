package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

public class CMD_ReadyToIntakeAlgaeThree extends SequentialCommandGroup{
    public CMD_ReadyToIntakeAlgaeThree(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae, SUB_CoralHolder p_intake, 
        GlobalVariables p_variables){
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kIntake))
                    ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyAlgael3Eject))
                    ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgael3))
                    ,new CMD_WristInPosition(p_wrist)
                    ,new CMD_ElevatorInPosition(p_elevator)
                    ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyIntakeAlgael3))
                    ,new CMD_PivotInPosition(p_pivot)
                )
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
