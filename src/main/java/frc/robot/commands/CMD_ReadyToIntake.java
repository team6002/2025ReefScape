package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntake extends SequentialCommandGroup{
    public CMD_ReadyToIntake(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake){
        addCommands(
            new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntake))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kIntake))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kIntake))
            ,new InstantCommand(()-> p_intake.setVoltage(CoralHolderConstants.kIntake))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new CMD_PivotInPosition(p_pivot)
            ,new CMD_WristInPosition(p_wrist)
        );
    }
}