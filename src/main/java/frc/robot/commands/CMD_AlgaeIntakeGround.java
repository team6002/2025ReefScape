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

public class CMD_AlgaeIntakeGround extends Command{
    private final SUB_GroundPivot m_groundpivot;
    private final SUB_Intake m_intake;
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setWrist;
    private boolean setPivot;
    private boolean setGroundPivot;

    private final Timer m_triggerTimer = new Timer();
    private final Timer m_SUCTimer = new Timer();
    private boolean isFinished = false;
        
    
        public CMD_AlgaeIntakeGround(SUB_Intake p_intake, SUB_GroundPivot p_groundPivot, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, GlobalVariables p_variables){
            m_groundpivot = p_groundPivot;
            m_intake = p_intake;
            m_pivot = p_pivot;
            m_elevator = p_elevator;
            m_flippyWrist = p_flippyWrist;
            m_variables = p_variables;
            m_spinnyWrist = p_spinnyWrist;
    
            addRequirements(m_pivot, m_elevator, m_flippyWrist, m_groundpivot, m_intake);
        }
    
        @Override
        public void initialize(){
            setElevator = false;
            setPivot = false;
            setGroundPivot = false;
            setWrist = false;
        
            m_triggerTimer.reset();
            m_triggerTimer.start();
            m_SUCTimer.reset();
            m_SUCTimer.stop();
            
            m_intake.setVoltage(IntakeConstants.kAlgaeIntake);
            m_intake.setCurrentLimit(40);
            isFinished = false;
        }
    
        @Override
        public void execute(){
            if(!setWrist){
                setWrist = true;
                m_flippyWrist.setGoal(FlippyWristConstants.kAlgaeGround);
                m_spinnyWrist.setGoal(SpinnyWristConstants.kIntake);
            }
    
            if(!setElevator){
                setElevator = true;
                m_elevator.setGoal(ElevatorConstants.kAlgaeGround);
            }
    
            if (!setGroundPivot){
                setGroundPivot = true;
                m_groundpivot.setGoal(GroundPivotConstants.kAlgaeGround);
            }

            if(setWrist && setElevator && setGroundPivot && m_groundpivot.inPosition() && m_elevator.inPosition() && !setPivot){
                setPivot = true;
                m_pivot.setGoal(PivotConstants.kAlgaeGround);
            }
            
            if(setWrist && setElevator && setPivot && m_elevator.inPosition() && m_pivot.inPosition()){
                m_variables.setRobotState(RobotState.ALGAE_LEVEL_2);
                if (m_intake.hasCoral()){
                    isFinished = true;
                }
            }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){
    m_intake.setCurrentLimit(30);
    m_intake.setVoltage(IntakeConstants.kAlgaeHolding);
        if(interrupted){m_intake.setVoltage(IntakeConstants.kOff); return;}
        m_pivot.setGoal(PivotConstants.kAlgaeLvl2Hold);
    
    }
}
