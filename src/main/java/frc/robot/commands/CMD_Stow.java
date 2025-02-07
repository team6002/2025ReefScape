package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_Stow extends SequentialCommandGroup{
    public CMD_Stow(SUB_Elevator p_elevator, SUB_CoralHolder p_coralHolder, SUB_Wrist p_wrist, SUB_ElevatorPivot p_elevatorPivot){
        addCommands(
            new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kHolding))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kHome))
            ,new WaitCommand(.5)
            ,new InstantCommand(()-> p_elevatorPivot.setGoal(ElevatorPivotConstants.kHome))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kHome))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new CMD_PivotInPosition(p_elevatorPivot)
            ,new CMD_WristInPosition(p_wrist)
        );
    }
}