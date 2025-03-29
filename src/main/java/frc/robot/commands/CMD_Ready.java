package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_Ready extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;

    private boolean setPivot;
    private boolean setWrist;
    private boolean setElevator;

    public CMD_Ready(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_Intake p_intake, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_variables = p_variables;


        addRequirements(m_pivot, m_elevator, m_flippyWrist, m_intake);
    }

    @Override
    public void initialize(){
        setPivot = false;
        setElevator = false;
        setWrist = false;

        m_intake.setConveyorVoltage(IntakeConstants.kConveyorOff);

        if(GlobalVariables.m_haveAlgae){
            m_intake.setVoltage(IntakeConstants.kHolding);
        }else{
            m_intake.setVoltage(IntakeConstants.kOff);
        }
    }

    @Override
    public void execute(){
        if(m_pivot.getPosition() <= PivotConstants.kReady){
            if(!setPivot){
                m_pivot.setGoal(PivotConstants.kReady);
                setPivot = true;
            }

            if(m_pivot.inPosition(PivotConstants.kReady) &! setElevator){
                m_elevator.setGoal(ElevatorConstants.kReady);
                setElevator = true;
            }

            if(m_pivot.inPosition(PivotConstants.kReady) && m_elevator.inPosition(ElevatorConstants.kReady) &! setWrist){
                m_flippyWrist.setGoal(FlippyWristConstants.kReady);
                setWrist = true;
            }
        }else{
            if(!setElevator){
                m_elevator.setGoal(ElevatorConstants.kReady);
                setElevator = true;
            }

            if(!setWrist){
                m_flippyWrist.setGoal(FlippyWristConstants.kReady);
                setWrist = true;
            }

            if(m_elevator.inPosition(ElevatorConstants.kReady) && m_flippyWrist.inPosition(FlippyWristConstants.kReady) &! setPivot){
                m_pivot.setGoal(PivotConstants.kReady);
                setPivot = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        //finish once all parts are in the correct spot
        return setElevator && setPivot && setWrist && m_elevator.inPosition() && m_flippyWrist.inPosition() && m_pivot.inPosition();
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted) return;

        m_variables.setRobotState(RobotState.READY);
    }
}