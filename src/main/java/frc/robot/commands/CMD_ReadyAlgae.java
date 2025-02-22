package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_ReadyAlgae extends SequentialCommandGroup{
    public CMD_ReadyAlgae(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_intake.setVoltage(CoralHolderConstants.kHolding))
            ,new ConditionalCommand(
                new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgae)), 
                new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgael3)),
                ()-> p_variables.getAlgaeTarget() == AlgaeTarget.LEVEL_2    
            )
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgae))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyAlgae))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kHome))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReady))
            ,new CMD_PivotInPosition(p_pivot)
        );
    }
}