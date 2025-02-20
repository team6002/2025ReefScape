package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Deploy extends Command{
    SUB_Elevator m_elevator;
    SUB_Wrist m_wrist;
    SUB_Pivot m_pivot;
    SUB_CoralHolder m_intake;
    GlobalVariables m_variables;
    public CMD_Deploy(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, GlobalVariables p_variables){
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){ switch (GlobalVariables.m_targetCoralLevel) {
            case 1:
                new CMD_DeployLevelOne(m_intake).schedule();
                break;
            case 2:
                new CMD_DeployLevelTwo(m_intake, m_wrist).schedule();
                break;
            case 3:
                new CMD_DeployLevelThree(m_intake, m_wrist).schedule();
                break;
            case 4:
                new CMD_DeployLevelFour(m_intake, m_wrist).schedule();
                break;
            default:
                break;
        }}

    @Override
    public boolean isFinished(){
        return true;
    }
}
