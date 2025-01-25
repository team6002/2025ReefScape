package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.Arm.SUB_Arm;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_ReadyToDeploy extends SequentialCommandGroup{
    public CMD_ReadyToDeploy(SUB_Elevator p_elevator, SUB_CoralHolder p_coralHolder, SUB_Arm p_arm, SUB_ElevatorPivot p_elevatorPivot){
        addCommands(
            new InstantCommand(()-> p_elevatorPivot.setGoal(ElevatorPivotConstants.kDeploy))
            ,new CMD_PivotInPosition(p_elevatorPivot)
            ,new InstantCommand(()-> p_arm.setReference(ArmConstants.kDeploy))
            ,new CMD_ElevatorSetDeploy(p_elevator)
        );
    }
}