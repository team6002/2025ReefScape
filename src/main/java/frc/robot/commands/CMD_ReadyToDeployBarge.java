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

public class CMD_ReadyToDeployBarge extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setWrist;
    private boolean setPivot;


    public CMD_ReadyToDeployBarge(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist,
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
        setElevator = false;
        setPivot = false;
        setWrist = false;
    }

    @Override
    public void execute(){
        if(!setPivot){
            m_pivot.setGoal(PivotConstants.kDeployBarge);
            setPivot = true;
        }

        if(setPivot && m_pivot.inPosition() && !setElevator){
            m_elevator.setGoal(ElevatorConstants.kDeployBarge);
            setElevator = true;
        }

        if(setPivot && m_pivot.inPosition() && !setWrist){
            m_flippyWrist.setGoal(FlippyWristConstants.kDeployBarge);
            m_spinnyWrist.setGoal(SpinnyWristConstants.kDeployFront);
            setWrist = true;
        }
    }

    @Override
    public boolean isFinished(){
        return setElevator && setPivot && setWrist && m_elevator.inPosition() && m_pivot.inPosition() && m_flippyWrist.inPosition();
    }

    @Override
    public void end(boolean interrupted){

        if(interrupted){return;}

        m_variables.setRobotState(RobotState.BARGE);
    }
}
