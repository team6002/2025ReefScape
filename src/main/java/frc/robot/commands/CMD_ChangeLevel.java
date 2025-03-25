package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ChangeLevel extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final GlobalVariables m_variables;
    private int m_newLevel;

    private boolean setElevator;
    private boolean setWrist;
    
    public CMD_ChangeLevel(SUB_Elevator p_elevator, SUB_Wrist p_wrist, GlobalVariables p_variables, int p_newLevel){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_variables = p_variables;
        m_newLevel = p_newLevel;
        addRequirements(m_elevator, m_wrist);
    }

    @Override
    public void initialize(){
        if(m_variables.isRobotState(RobotState.READY_TO_DEPLOY) || m_variables.isRobotState(RobotState.READY_TO_SCORE)){}
        else{return;}

        setElevator = false;
        setWrist = false;
        GlobalVariables.m_targetCoralLevel = m_newLevel;
        m_wrist.setGoal(WristConstants.kReadyToScore);
    }

    @Override
    public void execute(){
        if(m_wrist.inPosition(WristConstants.kReadyToScore) &! setElevator){
            setElevator = true;
            switch (GlobalVariables.m_targetCoralLevel) {
                case 2:
                    m_elevator.setGoal(ElevatorConstants.kDeployL2);
                    break;
                case 3:
                    m_elevator.setGoal(ElevatorConstants.kDeployL3);
                    break;
                case 4:
                    m_elevator.setGoal(ElevatorConstants.kDeployL4);
                    break;
                default:
                    break;
            }
        }

        if(setElevator &! setWrist && m_elevator.inPosition()
        ){
            setWrist = true;
            switch (GlobalVariables.m_targetCoralLevel) {
                case 2:
                    m_wrist.setGoal(WristConstants.kDeployL2);
                    break;
                case 3:
                    m_wrist.setGoal(WristConstants.kDeployL3);
                    break;
                case 4:
                    m_wrist.setGoal(WristConstants.kDeployL4);
                    break;
                default:
                    break;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return m_elevator.inPosition() && m_wrist.inPosition() && setElevator && setWrist;
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
    }
}
