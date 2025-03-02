package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetDeploy extends SequentialCommandGroup{

    public CMD_SetDeploy(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake,
        SUB_Algae p_algae, GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setRobotState(RobotState.DEPLOY))
            ,new CMD_Deploy(p_wrist, p_intake)
            ,new ConditionalCommand(
                new CMD_AlgaeLevel3Eject(p_intake, p_wrist, p_algae, p_elevator, p_variables, p_pivot)
                ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kStowing)).andThen(new InstantCommand(()-> p_algae.setReference(0)))
                ,()-> GlobalVariables.lvl3AlgaeException && GlobalVariables.m_targetCoralLevel == 2
            )
            ,new InstantCommand(()-> GlobalVariables.m_coralException = false)
            ,new InstantCommand(()-> GlobalVariables.m_haveCoral = false)
        );
    }
}
