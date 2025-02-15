package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeIntakeCoral extends SequentialCommandGroup{
    public CMD_AlgaeIntakeCoral(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(WristConstants.kAlgaeCoral))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kAlgaeCoral))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator).withTimeout(2)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kAlgaeCoral))
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kIntake))
            ,new CMD_PivotInPosition(p_pivot)
        );
    }
}
