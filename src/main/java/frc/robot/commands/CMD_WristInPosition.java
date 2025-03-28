package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;

public class CMD_WristInPosition extends Command{
    SUB_FlippyWrist m_flippyWrist;
    public CMD_WristInPosition(SUB_FlippyWrist p_flippyWrist){
        m_flippyWrist = p_flippyWrist;
    }

    @Override
    public boolean isFinished(){
        return m_flippyWrist.inPosition();
    }
}
