package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToDeployBarge extends SequentialCommandGroup{
    public CMD_ReadyToDeployBarge(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator){
        addCommands(
            new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kDeployBarge))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kDeployBarge))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kDeployBarge))
        );
    }
}
