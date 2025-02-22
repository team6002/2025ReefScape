package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToDeployLevelOneException extends SequentialCommandGroup{
    public CMD_ReadyToDeployLevelOneException(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyToScore))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kDeployl1Exception))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kDeployl1Exception))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kDeployl1Exception))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator).withTimeout(2)
        );
    }
}
