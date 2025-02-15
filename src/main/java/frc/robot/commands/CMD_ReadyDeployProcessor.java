package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyDeployProcessor extends SequentialCommandGroup{
    public CMD_ReadyDeployProcessor(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_ElevatorPivot p_pivot){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(WristConstants.kAlgaeProcessor))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kAlgaeProcessor))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kAlgaeProcessor))
            ,new CMD_PivotInPosition(p_pivot)
        );
    }
}
