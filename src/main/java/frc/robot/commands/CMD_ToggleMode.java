package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ToggleMode extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_CoralHolder m_intake;
    private final GlobalVariables m_variables;
    public CMD_ToggleMode(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, 
        GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        GlobalVariables.m_defenseMode = !GlobalVariables.m_defenseMode;

        if(!m_variables.isRobotState(RobotState.READY) || !m_variables.isRobotState(RobotState.READY_STOWED)){
            return;
        }

        if(GlobalVariables.m_defenseMode){
            new CMD_ReadyDefensive(m_elevator, m_wrist, m_pivot, m_intake).schedule();
        }else{
            new CMD_Ready(m_elevator, m_wrist, m_pivot, m_intake).schedule();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
