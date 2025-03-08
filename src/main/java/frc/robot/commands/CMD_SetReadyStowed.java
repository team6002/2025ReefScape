package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyStowed extends SequentialCommandGroup{

    public CMD_SetReadyStowed(SUB_Elevator p_elevator, SUB_Pivot p_pivot, SUB_Wrist p_wrist, SUB_CoralHolder p_intake, SUB_Algae p_algae,
        GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> GlobalVariables.m_coralException = false)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_STOWED))
            ,new CMD_SetReady(p_elevator, p_wrist, p_pivot, p_intake, p_algae)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_STOWED))
            ,new InstantCommand(()-> GlobalVariables.m_haveCoral = true)
        );
    }
}
