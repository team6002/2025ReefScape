package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ChangeAlgaeLevel extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_CoralHolder m_intake;
    private final SUB_Algae m_algae;
    private final GlobalVariables m_variables;
    private int m_newLevel;
    public CMD_ChangeAlgaeLevel(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, SUB_Algae p_algae, GlobalVariables p_variables, int p_level){
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_algae = p_algae;
        m_variables = p_variables;
        m_newLevel = p_level;
    }

    @Override
    public void initialize(){
        //update new level if not already there
        if(GlobalVariables.m_targetAlgaeLevel == m_newLevel){
            return;
        }

        //check if in good state or if already at the selected level, change value but do not move robot
        if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY) == false){
            GlobalVariables.m_targetAlgaeLevel = m_newLevel;
            return;
        }

        if(GlobalVariables.m_targetAlgaeLevel == 2){
            m_pivot.setGoal(PivotConstants.kReadyAlgae);
        }else{
            m_pivot.setGoal(PivotConstants.kReadyAlgael3);
        }
        GlobalVariables.m_targetAlgaeLevel = m_newLevel;

        if(m_newLevel == 2){
            m_variables.setAlgaeTarget(AlgaeTarget.LEVEL_2);
        }else if(m_newLevel == 3){
            m_variables.setAlgaeTarget(AlgaeTarget.LEVEL_3);
        }

        new CMD_Algae(m_wrist, m_pivot, m_elevator, m_algae, m_intake, m_variables);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
