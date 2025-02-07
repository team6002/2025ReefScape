package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_WristSetDeploy extends Command{
    private final SUB_Wrist m_wrist;

    CMD_WristSetDeploy(SUB_Wrist p_wrist){
        m_wrist = p_wrist;
    }

    @Override
    public void initialize(){
        switch (GlobalVariables.m_targetLevel) {
            case 1:
                m_wrist.setGoal(WristConstants.kDeployl1);
                break;
            case 2:
                m_wrist.setGoal(WristConstants.kDeployl2);
                break;
            case 3:
                m_wrist.setGoal(WristConstants.kDeployl3);
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return m_wrist.inPosition();
    }
}
