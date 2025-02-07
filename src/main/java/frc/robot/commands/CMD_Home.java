package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.*;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_Home extends SequentialCommandGroup{
    public CMD_Home(SUB_Elevator p_elevator, SUB_CoralHolder p_coralHolder, SUB_Wrist p_wrist, SUB_ElevatorPivot p_elevatorPivot
                    , GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setRobotState(RobotState.TRANSITIONING_TO_HOME))
            ,new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kHolding))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kStowing))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kHome))
            ,new CMD_ElevatorInPosition(p_elevator)
            // ,new InstantCommand(()-> p_elevatorPivot.setGoal(ElevatorPivotConstants.kHome))
            // ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kHome))
            // ,new CMD_PivotInPosition(p_elevatorPivot)
            // ,new CMD_WristInPosition(p_wrist)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.HOME))
        );
    }
}
