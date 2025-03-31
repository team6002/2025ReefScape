package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_ReadyToDeployProcessor extends Command{
    private final SUB_GroundPivot m_groundpivot;
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setWrist;
    private boolean setPivot;
    private boolean setGroundPivot;

    public CMD_ReadyToDeployProcessor(SUB_GroundPivot p_groundPivot, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, GlobalVariables p_variables){
        m_groundpivot = p_groundPivot;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_variables = p_variables;

        addRequirements(m_pivot, m_elevator, m_flippyWrist);
    }

    @Override
    public void initialize(){
        setElevator = false;
        setPivot = false;
        setGroundPivot = false;
        setWrist = false;
    }

    @Override
    public void execute(){
        if(!setWrist){
            setWrist = true;
            m_flippyWrist.setGoal(FlippyWristConstants.kAlgaeProcessor);
        }

        if(!setElevator){
            setElevator = true;
            m_elevator.setGoal(ElevatorConstants.kAlgaeProcessor);
        }

        if (!setGroundPivot){
            setGroundPivot = true;
            m_groundpivot.setGoal(GroundPivotConstants.kProcessor);
        }
        if(setWrist && setElevator && m_flippyWrist.inPosition() && m_elevator.inPosition() &! setPivot){
            setPivot = true;
            m_pivot.setGoal(PivotConstants.kAlgaeProcessor);
        }
    }

    @Override
    public boolean isFinished(){
        return setElevator && setPivot && setWrist && m_elevator.inPosition() && m_pivot.inPosition() && m_flippyWrist.inPosition();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted){return;}

        m_variables.setRobotState(RobotState.PROCESSOR);
    }
}
