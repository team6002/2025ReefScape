package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Arm.SUB_Arm;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_Score extends Command{
    SUB_Elevator m_elevator;
    SUB_Arm m_arm;
    SUB_CoralHolder m_coralHolder;
    GlobalVariables m_variables;
    public CMD_Score(SUB_Elevator p_elevator, SUB_Arm p_arm, SUB_CoralHolder p_coralHolder, GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_arm = p_arm;
        m_coralHolder = p_coralHolder;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch(m_variables.getRobotState()){
           case HOME:
                new CMD_ReadyToIntake(m_elevator, m_coralHolder, m_variables).schedule();
                m_variables.setRobotState(RobotState.READY_TO_INTAKE);
                break;
            case READY_TO_INTAKE:
                //TODO: schedule command
                m_variables.setRobotState(RobotState.STOW);
                break;
           case STOW:
                //TODO: schedule command
                m_variables.setRobotState(RobotState.READY_TO_DEPLOY);
                break;
            case READY_TO_DEPLOY:
                //TODO: schedule command
                m_variables.setRobotState(RobotState.HOME);
                break;
            case DEPLOY:
                //TODO: schedule command
                m_variables.setRobotState(RobotState.HOME);
                break;
            default:
                break;
        }
    }
}
