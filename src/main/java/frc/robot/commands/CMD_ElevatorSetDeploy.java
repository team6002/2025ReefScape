package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_ElevatorSetDeploy extends Command{
    private final SUB_Elevator m_elevator;
    public CMD_ElevatorSetDeploy(SUB_Elevator p_elevator){
        m_elevator = p_elevator;
    }

    @Override
    public void initialize(){
        switch (GlobalVariables.m_targetCoralLevel) {
            case 1:
                m_elevator.setGoal(ElevatorConstants.kDeployL1);
                break;
            case 2:
                m_elevator.setGoal(ElevatorConstants.kDeployL2);
                break;
            case 3:
                m_elevator.setGoal(ElevatorConstants.kDeployL3);
                break;
            case 4:
                m_elevator.setGoal(ElevatorConstants.kDeployL4);
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return m_elevator.inPosition();
    }
}
