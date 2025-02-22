package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.Mode;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyHome extends Command{
    SUB_Elevator m_elevator;
    SUB_Wrist m_wrist;
    SUB_Pivot m_pivot;
    SUB_CoralHolder m_intake;
    GlobalVariables m_variables;
    public CMD_SetReadyHome(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, GlobalVariables p_variables){
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        if(m_variables.isMode(Mode.DEFENSIVE)){
            new CMD_ReadyHomeDefensive(m_elevator, m_wrist, m_pivot, m_intake).schedule();
        }else{
            new CMD_ReadyHome(m_elevator, m_wrist, m_pivot, m_intake).schedule();
        }
    }
}
