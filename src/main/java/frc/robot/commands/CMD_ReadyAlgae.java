package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_ReadyAlgae extends SequentialCommandGroup{
    public CMD_ReadyAlgae(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_ElevatorPivot p_pivot, SUB_CoralHolder p_coralHolder, 
        SUB_Algae p_algae){
        addCommands(
            new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kHolding))
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kHolding))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgae))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyAlgae))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReady))
            ,new CMD_PivotInPosition(p_pivot)
        );
    }
}