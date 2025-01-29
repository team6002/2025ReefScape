package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_Score extends Command{
    SUB_Elevator m_elevator;
    SUB_Wrist m_wrist;
    SUB_CoralHolder m_coralHolder;
    SUB_ElevatorPivot m_elevatorPivot;
    GlobalVariables m_variables;
    public CMD_Score(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_CoralHolder p_coralHolder, SUB_ElevatorPivot p_elevatorPivot, 
                     GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_coralHolder = p_coralHolder;
        m_elevatorPivot = p_elevatorPivot;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch(m_variables.getRobotState()){
           case HOME:
                new CMD_ReadyToIntake(m_elevator, m_coralHolder, m_wrist, m_elevatorPivot).schedule();
                m_variables.setRobotState(RobotState.READY_TO_INTAKE);
                break;
            case READY_TO_INTAKE:
                new CMD_Stow(m_elevator, m_coralHolder, m_wrist, m_elevatorPivot).schedule();
                m_variables.setRobotState(RobotState.STOW);
                break;
           case STOW:
                new CMD_ReadyToDeploy(m_elevator, m_coralHolder, m_wrist, m_elevatorPivot).schedule();
                m_variables.setRobotState(RobotState.READY_TO_DEPLOY);
                break;
            case READY_TO_DEPLOY:
                new CMD_Deploy(m_coralHolder).schedule();
                m_variables.setRobotState(RobotState.DEPLOY);
                break;
            case DEPLOY:
                new CMD_Home(m_elevator, m_coralHolder, m_wrist, m_elevatorPivot).schedule();
                m_variables.setRobotState(RobotState.HOME);
                break;
            default:
                break;
        }
    }
}
