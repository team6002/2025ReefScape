package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_Home extends SequentialCommandGroup{
    public CMD_Home(SUB_Elevator p_elevator, SUB_CoralHolder p_intake, SUB_Wrist p_wrist, SUB_Pivot p_pivot){
        addCommands(
            new InstantCommand(()-> p_intake.setVoltage(CoralHolderConstants.kHolding))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kHome))
            ,new CMD_ElevatorReset(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kHome))
            ,new CMD_PivotInPosition(p_pivot)
            ,new CMD_WristInPosition(p_wrist)
        );
    }
}
