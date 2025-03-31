package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_ChangeLevelThree extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_Pivot m_pivot;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setPivot;
    
    public CMD_ChangeLevelThree(SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_Pivot p_pivot, GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist; 
        m_pivot = p_pivot;
        m_variables = p_variables;
        addRequirements(m_elevator, m_flippyWrist, m_pivot);
    }

    @Override
    public void initialize(){
        GlobalVariables.m_targetCoralLevel = 3;

        if(!m_variables.isRobotState(RobotState.READY_TO_DEPLOY) &! m_variables.isRobotState(RobotState.DEPLOY)){
            return;
        }

        setElevator = false;
        setPivot = false;
        if(GlobalVariables.m_placeFront) m_flippyWrist.setGoal(FlippyWristConstants.kDeployFrontL3);
        else m_flippyWrist.setGoal(FlippyWristConstants.kDeployL3);
    }
       

    @Override
    public void execute(){
        if(GlobalVariables.m_placeFront){
            if(!setElevator){
                m_elevator.setGoal(ElevatorConstants.kDeployFrontL3);
                setElevator = true;
            }
    
            if(!setPivot && m_elevator.inPosition(ElevatorConstants.kDeployFrontL3)){
                m_pivot.setGoal(PivotConstants.kDeployFrontL3);
                setPivot = true;
            }
        }else{
            if(!setElevator){
                m_elevator.setGoal(ElevatorConstants.kDeployL3);
                setElevator = true;
            }
    
            if(!setPivot){
                m_pivot.setGoal(PivotConstants.kDeployL3);
                setPivot = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return m_elevator.inPosition() && m_pivot.inPosition() && setElevator && setPivot;
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
    }
}
