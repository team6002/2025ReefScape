package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Winch.SUB_Winch;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyClimb extends SequentialCommandGroup{
    
    public CMD_SetReadyClimb(SUB_Pivot p_pivot, SUB_Wrist p_wrist, SUB_Elevator p_elevator, SUB_Winch p_winch, SUB_GroundPivot p_groundPivot){
        addCommands(
            new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kClimb))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kClimb))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_groundPivot.setGoal(GroundPivotConstants.kHome))
            ,new CMD_GroundPivotInPosition(p_groundPivot)
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorReset(p_elevator)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kClimb))
            ,new InstantCommand(()-> p_winch.setReference(WinchConstants.kReadyClimb))
        );
    }
}
