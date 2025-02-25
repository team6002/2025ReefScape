package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Exception extends Command{
    private final SUB_Wrist m_wrist;
    private final SUB_Elevator m_elevator;
    private final SUB_Pivot m_pivot;
    private final SUB_CoralHolder m_intake;
    private final GlobalVariables m_variables;
    public CMD_Exception(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_CoralHolder p_intake,
        GlobalVariables p_variables){

        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_intake = p_intake;
        m_wrist = p_wrist;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        GlobalVariables.m_algaeExceptionMode = !GlobalVariables.m_algaeExceptionMode;

        if(GlobalVariables.m_algaeExceptionMode){
            if(m_variables.isRobotState(RobotState.READY_TO_INTAKE)){
                //if coral is in the middle of robot & coral chute
                new CMD_ReadyToIntakeException(m_pivot, m_elevator, m_wrist).schedule();
                return;
            }

            if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
                new CMD_ReadyToDeployException(m_elevator, m_wrist, m_pivot, m_intake, m_variables).schedule();
                return;
            }   
        }else{
            if(m_variables.isRobotState(RobotState.READY_TO_INTAKE)){
                //if coral is in the middle of robot & coral chute
                new CMD_ReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake).schedule();
                return;
            }
    
            if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
                new CMD_ReadyToDeploy(m_elevator, m_wrist, m_pivot, m_intake, m_variables).schedule();
                return;
            }
        }
        
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
