package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.SUB_Arm;

public class CMD_ArmInPosition extends Command{
    SUB_Arm m_arm;
    public CMD_ArmInPosition(SUB_Arm p_arm){
        m_arm = p_arm;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(m_arm.getGoal() - m_arm.getPosition()) < 1;
    }
}
