package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntakeAlgae extends Command{
    SUB_Wrist m_wrist;
    SUB_Pivot m_pivot;
    SUB_Elevator m_elevator;
    SUB_Algae m_algae;
    GlobalVariables m_variables;
    public CMD_ReadyToIntakeAlgae(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae,
        GlobalVariables p_variables){
        
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_algae = p_algae;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch (m_variables.getAlgaeTarget()) {
            case LEVEL_2:
                new CMD_ReadyToIntakeAlgaeTwo(m_wrist, m_pivot, m_elevator, m_algae).schedule();
                break;
            case LEVEL_3:
                new CMD_ReadyToIntakeAlgaeThree(m_wrist, m_pivot, m_elevator, m_algae).schedule();
                break;
            case GROUND:
                new CMD_AlgaeIntakeGround(m_wrist, m_pivot, m_elevator, m_algae).schedule();
                break;
            case CORAL:
                new CMD_AlgaeIntakeCoral(m_wrist, m_pivot, m_elevator, m_algae).schedule();
            default:
                break;
        }

    }

    @Override
    public boolean isFinished(){
        return m_wrist.inPosition() && m_pivot.inPosition() && m_elevator.inPosition();
    }
}
