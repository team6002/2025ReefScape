package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntakeAlgaeGround extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Algae m_algae;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setWrist;
    private boolean setPivot;

    private final Timer m_triggerTimer = new Timer();
    private boolean isFinished = false;


    public CMD_ReadyToIntakeAlgaeGround(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Algae p_algae, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_algae = p_algae;
        m_variables = p_variables;

        addRequirements(m_pivot, m_elevator, m_wrist, m_algae);
    }

    @Override
    public void initialize(){
        setElevator = false;
        setPivot = false;
        setWrist = false;

        m_triggerTimer.reset();
        m_triggerTimer.start();
        isFinished = false;

        m_algae.setReference(AlgaeConstants.kIntake);
    }

    @Override
    public void execute(){
        if(!setWrist){
            setWrist = true;
            m_wrist.setGoal(WristConstants.kIntakeAlgaeGround);
        }

        if(!setElevator){
            setElevator = true;
            m_elevator.setGoal(ElevatorConstants.kIntakeAlgaeGround);
        }

        if(setWrist && setElevator && m_wrist.inPosition() && m_elevator.inPosition() &! setPivot){
            setPivot = true;
            m_pivot.setGoal(PivotConstants.kIntakeAlgaeGround);
        }

        if(setWrist && setElevator && setPivot && m_wrist.inPosition() && m_elevator.inPosition() && m_pivot.inPosition()){
            m_variables.setRobotState(RobotState.GROUND);
            if(m_algae.getCurrent() > 20){
                if(m_triggerTimer.get() > .1){
                    isFinished = true;
                    GlobalVariables.m_haveAlgae = true;
                }
            }else{
                m_triggerTimer.reset();
            }   
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){

        if(interrupted){m_algae.setReference(AlgaeConstants.kOff); return;}

        m_algae.setReference(AlgaeConstants.kHolding);
    }
}
