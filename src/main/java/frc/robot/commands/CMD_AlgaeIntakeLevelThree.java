package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_AlgaeIntakeLevelThree extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setWrist;
    private boolean setPivot;

    private final Timer m_triggerTimer = new Timer();
    private final Timer m_SUCTimer = new Timer();
    private boolean isFinished = false;


    public CMD_AlgaeIntakeLevelThree(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_Intake p_intake, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_variables = p_variables;

        addRequirements(m_pivot, m_elevator, m_flippyWrist, m_intake);
    }

    @Override
    public void initialize(){
        setElevator = false;
        setPivot = false;
        setWrist = false;

        m_triggerTimer.reset();
        m_triggerTimer.start();
        m_SUCTimer.reset();
        m_SUCTimer.stop();
        isFinished = false;

        // m_intake.setReference(IntakeConstants.kAlgaeIntake);
        m_intake.setVoltage(8);
        m_intake.setCurrentLimit(30);
    }

    @Override
    public void execute(){
        if(!setWrist){
            setWrist = true;
            m_flippyWrist.setGoal(FlippyWristConstants.kReadyAlgael3);
        }

        if(setWrist &! setElevator){
            setElevator = true;
            m_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgael3);
        }

        if(setWrist && setElevator && m_flippyWrist.inPosition() && m_elevator.inPosition() &! setPivot){
            setPivot = true;
            m_pivot.setGoal(PivotConstants.kReadyIntakeAlgael3);
        }

        if(setWrist && setElevator && setPivot && m_elevator.inPosition() && m_pivot.inPosition()){
            m_variables.setRobotState(RobotState.ALGAE_LEVEL_3);
            if(m_intake.getCurrent() > 20){
                if(m_triggerTimer.get() > .1){
                    // isFinished = true;
                    GlobalVariables.m_haveAlgae = true;
                    m_SUCTimer.start();
                }
            }else{
                m_triggerTimer.reset();
            }   
        }
        if (m_SUCTimer.get() >= .3){
            // m_flippyWrist.setConstraints(FlippyWristConstants.kAlgaeVel, FlippyWristConstants.kAlgaeAccel);
            // if (m_flippyWrist.inPosition()){
                isFinished = true;
            // }
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setCurrentLimit(30);
        if(interrupted){m_intake.setVoltage(IntakeConstants.kOff); return;}
        m_flippyWrist.setGoal(FlippyWristConstants.kAlgaeHolding);
        m_elevator.setGoal(ElevatorConstants.kIntakedAlgael3);
        m_intake.setVoltage(IntakeConstants.kAlgaeHolding);
    }
}
