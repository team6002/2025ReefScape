package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;

public class CMD_ReadyToIntakeAlgaeThree extends SequentialCommandGroup{
    public CMD_ReadyToIntakeAlgaeThree(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae){
        addCommands(
            new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgae))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgael3))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyAlgael3))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyIntakeAlgael3))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kIntake))
        );
    }
}
