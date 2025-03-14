package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;

public class CMD_GroundPivotInPosition extends Command{
    private final SUB_GroundPivot m_groundPivot;
    public CMD_GroundPivotInPosition(SUB_GroundPivot p_groundPivot){
        m_groundPivot = p_groundPivot;
    }

    @Override
    public boolean isFinished(){
        return m_groundPivot.inPosition();
    }
}
