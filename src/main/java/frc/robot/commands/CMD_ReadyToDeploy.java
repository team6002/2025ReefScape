package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_ReadyToDeploy extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setPivot;
    private boolean setFlippyWrist;
    private boolean setSpinnyWrist;

    public CMD_ReadyToDeploy(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist,
        GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_spinnyWrist = p_spinnyWrist;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist);
    }

    @Override
    public void initialize(){
        setPivot = false;
        setElevator = false;
        setFlippyWrist = false; 
        setSpinnyWrist = false;

        GlobalVariables.m_targetCoralLevel = 3;
        m_variables.setRobotState(RobotState.READY_TO_DEPLOY);
    }

    @Override
    public void execute(){
        if(!m_pivot.inPosition(PivotConstants.kDeployL3) && !setPivot){
            m_pivot.setGoal(PivotConstants.kDeployL3);
            setPivot = true;
        }

        if(!m_elevator.inPosition(ElevatorConstants.kDeployL3) &! setElevator){
            m_elevator.setGoal(ElevatorConstants.kDeployL3);
            setElevator = true;
        }

        if(!m_flippyWrist.inPosition(FlippyWristConstants.kDeployL3) &! setFlippyWrist){
            m_flippyWrist.setGoal(FlippyWristConstants.kDeployL3);
            setFlippyWrist = true;
        }

        if(m_flippyWrist.inPosition(FlippyWristConstants.kDeployL3) &! setSpinnyWrist){
            m_spinnyWrist.setGoal(SpinnyWristConstants.kHome);
            setSpinnyWrist = true;
        }
    }

    @Override
    public boolean isFinished(){
        return setPivot && setElevator && setFlippyWrist && setSpinnyWrist && m_elevator.inPosition() && m_pivot.inPosition() && m_flippyWrist.inPosition() && m_spinnyWrist.inPosition();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
    }
}
