package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntakeException extends SequentialCommandGroup{
    public CMD_ReadyToIntakeException(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist){
        addCommands(
            new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntakeException))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kIntakeException))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kIntakeException))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new CMD_PivotInPosition(p_pivot)
            ,new CMD_WristInPosition(p_wrist)
        );
    }
}
