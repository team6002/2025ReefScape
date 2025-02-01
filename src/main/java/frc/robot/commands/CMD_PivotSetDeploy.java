package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_PivotSetDeploy extends Command{
    private final SUB_ElevatorPivot m_elevatorPivot;
    public CMD_PivotSetDeploy(SUB_ElevatorPivot p_elevatorPivot){
        m_elevatorPivot = p_elevatorPivot;
    }

    @Override
    public void initialize(){
        switch (GlobalVariables.m_targetLevel) {
            case 1:
                m_elevatorPivot.setGoal(ElevatorPivotConstants.kDeployl1);
                break;
            case 2:
                m_elevatorPivot.setGoal(ElevatorPivotConstants.kDeployl2);
                break;
            case 3:
                m_elevatorPivot.setGoal(ElevatorPivotConstants.kDeployl3);
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return Math.abs(m_elevatorPivot.getGoal() - m_elevatorPivot.getPosition()) < 1;
    }
}
