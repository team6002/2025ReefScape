package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_WristInPosition extends Command{
    SUB_Wrist m_wrist;
    public CMD_WristInPosition(SUB_Wrist p_wrist){
        m_wrist = p_wrist;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(m_wrist.getGoal() - m_wrist.getPosition()) < Math.toRadians(1);
    }
}
