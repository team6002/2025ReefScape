package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_PivotInPosition extends Command{
    SUB_Pivot m_pivot;
    public CMD_PivotInPosition(SUB_Pivot p_pivot){
        m_pivot = p_pivot;
    }

    @Override
    public boolean isFinished(){
        return m_pivot.inPosition();
    }
}
