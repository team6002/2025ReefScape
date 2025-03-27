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

public class CMD_ChangeLevelTwo extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setWrist;
    private boolean setPivot;
    
    public CMD_ChangeLevelTwo(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_variables = p_variables;
        addRequirements(m_elevator, m_wrist, m_pivot);
    }

    @Override
    public void initialize(){
        if(!m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            return;
        }

        setElevator = false;
        setWrist = false;
        setPivot = false;
        GlobalVariables.m_targetCoralLevel = 2;
        m_wrist.setGoal(WristConstants.kReadyToScore);
    }

    @Override
    public void execute(){
        if(m_wrist.inPosition(WristConstants.kReadyToScore) &! setElevator){
            m_elevator.setGoal(ElevatorConstants.kDeployL2);
            setElevator = true;
        }

        if(setElevator &! setWrist && m_elevator.inPosition()){
            m_wrist.setGoal(WristConstants.kDeployL2);
            setWrist = true;
        }

        if(setElevator && setWrist && m_elevator.inPosition() && m_wrist.inPosition() &! setPivot){
            m_pivot.setGoal(PivotConstants.kDeployL2);
            setPivot = true;
        }
    }

    @Override
    public boolean isFinished(){
        return m_elevator.inPosition() && m_wrist.inPosition() && m_pivot.inPosition() && setElevator && setWrist && setPivot;
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
    }
}
