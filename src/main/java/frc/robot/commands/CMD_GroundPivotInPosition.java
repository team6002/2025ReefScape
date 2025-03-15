package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;

public class CMD_GroundPivotInPosition extends Command{
    SUB_GroundPivot m_GroundPivot;
    public CMD_GroundPivotInPosition(SUB_GroundPivot p_GroundPivot){
        m_GroundPivot = p_GroundPivot;
    }

    @Override
    public boolean isFinished(){
        return m_GroundPivot.inPosition();
    }
}
