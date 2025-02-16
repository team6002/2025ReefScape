package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ChangeLevel extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final GlobalVariables m_variables;
    private int m_newLevel;
    public CMD_ChangeLevel(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, GlobalVariables p_variables, int p_level){
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_variables = p_variables;
        m_newLevel = p_level;
    }

    @Override
    public void initialize(){
        if(GlobalVariables.m_targetCoralLevel == m_newLevel){
            return;
        }else{
            GlobalVariables.m_targetCoralLevel = m_newLevel;
        }
        
        //check if in good state or if already at the selected level
        if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY) == false){
            return;
        }

        m_pivot.setGoal(PivotConstants.kReady);

        switch (m_newLevel) {
            case 1:
                new SequentialCommandGroup(
                    new CMD_PivotInPosition(m_pivot)
                    ,new CMD_ReadyToDeployLevelOne(m_elevator, m_wrist, m_pivot)
                ).schedule();
                break;
            case 2:
                new SequentialCommandGroup(
                    new CMD_PivotInPosition(m_pivot)
                    ,new CMD_ReadyToDeployLevelTwo(m_elevator, m_wrist, m_pivot)
                ).schedule();
                break;
            case 3:
            new SequentialCommandGroup(
                    new CMD_PivotInPosition(m_pivot)
                    ,new CMD_ReadyToDeployLevelThree(m_elevator, m_wrist, m_pivot)
                ).schedule();
                break;
            case 4:
            new SequentialCommandGroup(
                    new CMD_PivotInPosition(m_pivot)
                    ,new CMD_ReadyToDeployLevelFour(m_elevator, m_wrist, m_pivot)
                ).schedule();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
