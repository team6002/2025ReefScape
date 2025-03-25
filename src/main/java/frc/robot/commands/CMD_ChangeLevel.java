package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ChangeLevel extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final GlobalVariables m_variables;
    private int m_newLevel;
    private boolean wristSet;
    private boolean elevatorSet;
    
    public CMD_ChangeLevel(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, GlobalVariables p_variables,
        int p_newLevel){
        
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_variables = p_variables;
        m_newLevel = p_newLevel;
        addRequirements(m_elevator, m_pivot, m_wrist);
    }

    @Override
    public void initialize(){
        elevatorSet = false;
        wristSet = false;

        if(m_newLevel < 2 || m_newLevel >3) return;

        GlobalVariables.m_targetCoralLevel = m_newLevel;

        if(!m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            elevatorSet = true;
            wristSet = true;
            return;
        }else{
            elevatorSet = false;
            wristSet = false;
        }

        m_wrist.setGoal(WristConstants.kReadyToScore);
    }

    @Override
    public void execute(){
        if(m_wrist.inPosition() &! elevatorSet){
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
            elevatorSet = true;
        }

        if(m_wrist.inPosition(WristConstants.kReadyToScore) && m_elevator.inPosition() &! wristSet){
            wristSet = true;
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

        if(wristSet && elevatorSet) m_variables.setRobotState(RobotState.READY_TO_SCORE); return;
    }
}
