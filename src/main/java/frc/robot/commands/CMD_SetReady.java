package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReady extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_CoralHolder m_intake;
    private final SUB_Algae m_algae;
    public CMD_SetReady(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, SUB_Algae p_algae){
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_algae = p_algae;
    }

    @Override
    public void initialize(){
        m_algae.setReference(AlgaeConstants.kHolding);
        if(GlobalVariables.m_defenseMode){
            new CMD_ReadyDefensive(m_elevator, m_wrist, m_pivot, m_intake).schedule();
        }else{
            new CMD_Ready(m_elevator, m_wrist, m_pivot, m_intake).schedule();
        }
    }

    @Override
    public boolean isFinished(){
        return m_elevator.inPosition() && m_pivot.inPosition() && m_wrist.inPosition();
    }
}
