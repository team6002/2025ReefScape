package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.*;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntake extends SequentialCommandGroup{
    public CMD_ReadyToIntake(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_ElevatorPivot p_elevatorPivot, SUB_CoralHolder p_coralHolder
                             , GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setRobotState(RobotState.TRANSITIONING_TO_INTAKE))
            ,new InstantCommand(()-> p_elevatorPivot.setGoal(ElevatorPivotConstants.kIntake))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kIntake))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kIntake))
            ,new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kIntake))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_TO_INTAKE))
            ,new CMD_IntakeStow(p_coralHolder, p_wrist, p_elevatorPivot, p_elevator, p_coralHolder, p_variables)
            ,new CMD_Ready(p_elevator, p_wrist, p_elevatorPivot, p_coralHolder, p_variables)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_STOWED))
        );
    }
}