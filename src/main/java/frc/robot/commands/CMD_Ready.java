package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_Ready extends SequentialCommandGroup{
    public CMD_Ready(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_ElevatorPivot p_elevatorPivot
                             , GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setRobotState(RobotState.TRANSITIONING_TO_INTAKE))
            ,new InstantCommand(()-> p_elevatorPivot.setGoal(ElevatorPivotConstants.kIntake))
            ,new CMD_PivotInPosition(p_elevatorPivot)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kIntake))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kIntake))
            // ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY))
        );
    }
}