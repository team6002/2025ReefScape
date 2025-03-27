package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeReadyToDeploy extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setPivot;
    private boolean setWrist;

    public CMD_AlgaeReadyToDeploy(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_wrist);
    }

    @Override
    public void initialize(){
        setPivot = false;
        setElevator = false;
        setWrist = false;

        if(m_variables.isRobotState(RobotState.ALGAE_LEVEL_2)){
            m_pivot.setGoal(PivotConstants.kReadyAlgae);
        }
        if(m_variables.isRobotState(RobotState.ALGAE_LEVEL_3)){
            m_pivot.setGoal(PivotConstants.kReadyAlgael3);
        }
    }

    @Override
    public void execute(){
        if(!m_wrist.inPosition(WristConstants.kDeployL3) &! setWrist && m_pivot.inPosition()){
            m_wrist.setGoal(WristConstants.kDeployL3);
            setWrist = true;
        }

        if(m_wrist.inPosition(WristConstants.kDeployL3) &! setElevator){
            m_elevator.setGoal(ElevatorConstants.kDeployL3);
            setElevator  = true;
        }

        if(m_wrist.inPosition(WristConstants.kDeployL3) && m_elevator.inPosition(ElevatorConstants.kDeployL3) &! setPivot){
            m_pivot.setGoal(PivotConstants.kDeployL3);
            setPivot = true;
        }
    }

    @Override
    public boolean isFinished(){
        return setPivot && setElevator && setWrist && m_elevator.inPosition() && m_pivot.inPosition() && m_wrist.inPosition();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
        m_variables.setRobotState(RobotState.READY_TO_DEPLOY);
    }
}
