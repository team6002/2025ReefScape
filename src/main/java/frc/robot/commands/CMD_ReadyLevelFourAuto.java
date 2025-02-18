package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_ReadyLevelFourAuto extends SequentialCommandGroup{
    public CMD_ReadyLevelFourAuto(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake){
        addCommands(
            new InstantCommand(()-> p_intake.setReference(CoralHolderConstants.kHolding))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReady))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new WaitCommand(.2)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kDeployl4))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kDeployL4))
            ,new CMD_ElevatorInPosition(p_elevator)
        );
    }
}