package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToDeployLevelFour extends SequentialCommandGroup{
    public CMD_ReadyToDeployLevelFour(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyToScore))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kDeployl4))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kDeployL4))
            ,new CMD_ElevatorInPosition(p_elevator).withTimeout(2)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kDeployl4))
            ,new CMD_WristInPosition(p_wrist)
        );
    }
}
