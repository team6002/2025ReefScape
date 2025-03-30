package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_SpinnyWristInPosition extends Command{
    SUB_SpinnyWrist m_spinnyWrist;
    public CMD_SpinnyWristInPosition(SUB_SpinnyWrist p_spinnyWrist){
        m_spinnyWrist = p_spinnyWrist;
    }

    @Override
    public boolean isFinished(){
        return m_spinnyWrist.inPosition();
    }
}
