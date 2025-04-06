package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_IntakeArmGround extends Command{
    private final SUB_GroundPivot m_groundpivot;
    private final SUB_Intake m_intake;
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setFlippyWrist;
    private boolean setSpinnyWrist;
    private boolean setPivot;
    private boolean setGroundPivot;

    private final Timer m_triggerTimer = new Timer();
    // private final Timer m_SUCTimer = new Timer();
    private boolean isFinished = false;
        
    
        public CMD_IntakeArmGround(SUB_Intake p_intake, SUB_GroundPivot p_groundPivot, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_SpinnyWrist, GlobalVariables p_variables){
            m_groundpivot = p_groundPivot;
            m_intake = p_intake;
            m_pivot = p_pivot;
            m_elevator = p_elevator;
            m_flippyWrist = p_flippyWrist;
            m_spinnyWrist = p_SpinnyWrist;
            m_variables = p_variables;
    
            addRequirements(m_pivot, m_elevator, m_flippyWrist);
        }
    
        @Override
        public void initialize(){
            setElevator = false;
            setPivot = false;
            setGroundPivot = false;
            setFlippyWrist = false;
        
            m_triggerTimer.reset();
            m_triggerTimer.start();
            
            m_intake.setReference(IntakeConstants.kAlgaeIntake);
        }
    
        @Override
        public void execute(){
            if(!setFlippyWrist){
                setFlippyWrist = true;
                m_flippyWrist.setGoal(FlippyWristConstants.kIntakeGround);
            }

            if(!setSpinnyWrist){
                setSpinnyWrist = true;
                m_spinnyWrist.setGoal(SpinnyWristConstants.kIntake);
            }
    
            if(!setElevator){
                setElevator = true;
                m_elevator.setGoal(ElevatorConstants.kIntakeGround);
            }
    
            if (!setGroundPivot){
                setGroundPivot = true;
                m_groundpivot.setGoal(GroundPivotConstants.kClimb);
            }
            if(setFlippyWrist && setElevator && m_flippyWrist.inPosition() && m_elevator.inPosition() && !setPivot){
                setPivot = true;
                m_pivot.setGoal(PivotConstants.kIntakeGround);
            }
            
            if(setFlippyWrist && setElevator && setPivot && m_elevator.inPosition() && m_pivot.inPosition()){
                m_variables.setRobotState(RobotState.ALGAE_LEVEL_3);
                if(m_intake.getCurrent() > 20){
                    if(m_triggerTimer.get() > .1){
                        // isFinished = true;
                        GlobalVariables.m_haveCoral = true;
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
    // m_intake.setCurrentLimit(30);
    m_intake.setVoltage(IntakeConstants.kHolding);
        if(interrupted){m_intake.setVoltage(IntakeConstants.kHolding); return;}
        new CMD_Ready(m_pivot, m_elevator, m_flippyWrist, m_intake, m_variables).schedule();
    }
}
