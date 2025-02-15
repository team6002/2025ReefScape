package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_PivotSetDeploy extends Command{
    private final SUB_Pivot m_pivot;
    public CMD_PivotSetDeploy(SUB_Pivot p_pivot){
        m_pivot = p_pivot;
    }

    @Override
    public void initialize(){
        switch (GlobalVariables.m_targetCoralLevel) {
            case 1:
                m_pivot.setGoal(PivotConstants.kDeployl1);
                break;
            case 2:
                m_pivot.setGoal(PivotConstants.kDeployl2);
                break;
            case 3:
                m_pivot.setGoal(PivotConstants.kDeployl3);
                break;
            case 4:
                m_pivot.setGoal(PivotConstants.kDeployl4);
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return m_pivot.inPosition();
    }
}
