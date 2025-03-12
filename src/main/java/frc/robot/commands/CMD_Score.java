package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Score extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_CoralHolder m_intake;
    private final SUB_Pivot m_pivot;
    private final SUB_Algae m_algae;
    private final GlobalVariables m_variables;
    public CMD_Score(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_CoralHolder p_intake, SUB_Pivot p_pivot, SUB_Algae p_algae,
        GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_intake = p_intake;
        m_pivot = p_pivot;
        m_algae = p_algae;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        if(m_variables.getAlgaeTarget() == AlgaeTarget.GROUND && m_pivot.getGoal() <= PivotConstants.kIntakeAlgaeGround){
            new ParallelCommandGroup(
                new CMD_ReadyAlgae(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                ,new SequentialCommandGroup(
                    new CMD_CheckCoral(m_intake)
                    ,new ConditionalCommand(
                        new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_INTAKE))
                        ,new InstantCommand(()-> m_variables.setRobotState(RobotState.HOME))
                        ,()-> GlobalVariables.m_haveCoral
                    )
                ) 
            ).schedule();
            return;
        }
        switch (m_variables.getRobotState()) {
            case HOME:
                new SequentialCommandGroup(
                    new CMD_SetReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                    ,new CMD_Score(m_elevator, m_wrist, m_intake, m_pivot, m_algae, m_variables)
                ).schedule();
                break;
            case READY_TO_INTAKE:
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                    ,new InstantCommand(()-> GlobalVariables.m_targetCoralLevel = 3)
                    ,new CMD_ReadyToDeployLevelThree(m_elevator, m_wrist, m_pivot)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_DEPLOY))
                );
            case READY_TO_DEPLOY:
                new SequentialCommandGroup(
                    new CMD_SetDeploy(m_elevator, m_wrist, m_pivot, m_intake, m_algae, m_variables)
                    ,new CMD_Score(m_elevator, m_wrist, m_intake, m_pivot, m_algae, m_variables)
                ).schedule();
            case DEPLOY:
                new SequentialCommandGroup(
                    new CMD_SetReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                    ,new CMD_Score(m_elevator, m_wrist, m_intake, m_pivot, m_algae, m_variables)
                ).schedule();
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
