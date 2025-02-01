package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_ReadyToDeploy extends SequentialCommandGroup{
    public CMD_ReadyToDeploy(SUB_Elevator p_elevator, SUB_CoralHolder p_coralHolder, SUB_Wrist p_wrist, SUB_ElevatorPivot p_elevatorPivot
                             , GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyToScore))
            ,new CMD_PivotSetDeploy(p_elevatorPivot)
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorSetDeploy(p_elevator).withTimeout(2)
            ,new CMD_WristSetDeploy(p_wrist)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_TO_DEPLOY))
        );
    }
}