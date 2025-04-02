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

public class CMD_AlgaeReadyToDeploy extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setPivot;
    private boolean setWrist;

    public CMD_AlgaeReadyToDeploy(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_flippyWrist);
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
        if(!m_flippyWrist.inPosition(FlippyWristConstants.kDeployL3) && !setWrist && m_pivot.inPosition()){
            m_flippyWrist.setGoal(FlippyWristConstants.kDeployL3);
            setWrist = true;
        }

        if(m_flippyWrist.inPosition(FlippyWristConstants.kDeployL3) && !setElevator){
            m_elevator.setGoal(ElevatorConstants.kDeployL3);
            setElevator  = true;
        }

        if(m_flippyWrist.inPosition(FlippyWristConstants.kDeployL3) && m_elevator.inPosition(ElevatorConstants.kDeployL3) && !setPivot){
            m_pivot.setGoal(PivotConstants.kDeployL3);
            setPivot = true;
        }
    }

    @Override
    public boolean isFinished(){
        return setPivot && setElevator && setWrist && m_elevator.inPosition() && m_pivot.inPosition() && m_flippyWrist.inPosition();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
        m_variables.setRobotState(RobotState.READY_TO_DEPLOY);
    }
}
