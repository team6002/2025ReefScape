package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Configs.GroundIntake;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_IntakeHalf extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final SUB_Intake m_intake;
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_GroundIntake m_groundIntake;
    private final GlobalVariables m_variables;

    private final Timer m_intakeTimer = new Timer();
    private final Timer m_SUCTimer = new Timer();
    private final Timer m_groundIntakeTimer = new Timer();

    private boolean haveCoral;
    private boolean setPivot;
    private boolean setWrist;
    private boolean setElevator;
    private boolean setGroundPivot;
    private boolean setSpinnyWrist;
    private boolean transfered;
    private double prevIntakePosition;

    public CMD_IntakeHalf(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, 
        SUB_Intake p_intake, SUB_GroundPivot p_groundPivot, SUB_GroundIntake p_groundIntake, GlobalVariables p_variables){
            
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_spinnyWrist = p_spinnyWrist;
        m_intake = p_intake;
        m_groundPivot = p_groundPivot;
        m_groundIntake = p_groundIntake;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_intake, m_groundIntake, m_groundPivot);
    }

    @Override
    public void initialize(){
        haveCoral = false;
        setPivot = false;
        setElevator = false;
        setWrist = false;
        setSpinnyWrist = false;
        setGroundPivot = false;

        m_intake.setConveyorVoltage(IntakeConstants.kConveyorOff);
        m_groundIntake.setReference(GroundIntakeConstants.kIntake);
        m_groundPivot.setGoal(GroundPivotConstants.kIntake);

        if (GlobalVariables.m_haveAlgae){
            m_intake.setVoltage(IntakeConstants.kAlgaeHolding);
        }
        m_intakeTimer.reset();
        m_intakeTimer.stop();
        
        m_SUCTimer.reset();
        m_SUCTimer.stop();

        m_groundIntakeTimer.reset();
        m_groundIntakeTimer.stop();
        m_variables.setRobotState(RobotState.READY_TO_INTAKE);
        m_intake.setCurrentLimit(30);
        prevIntakePosition = m_intake.getPosition();
    }

    @Override
    public void execute(){
        if (m_groundIntake.hasCorral()){
            GlobalVariables.m_groundHasCoral = true;
        }

        if(m_pivot.getPosition() <= PivotConstants.kIntakeGround){
            if(!setPivot){
                if (GlobalVariables.m_haveAlgae){
                    m_pivot.setGoal(PivotConstants.kIntakeHalf);
                }else {
                    m_pivot.setGoal(PivotConstants.kIntakeGround);    
                }
            
                m_pivot.setGoal(PivotConstants.kIntakeGround);
                setPivot = true;
            }

            if(m_pivot.inPosition(PivotConstants.kIntakeGround) && !setElevator){
                m_elevator.setGoal(ElevatorConstants.kIntakeGround);
                setElevator = true;
            }

            if(m_pivot.inPosition(PivotConstants.kIntakeGround) && !setWrist){
                if (GlobalVariables.m_haveAlgae){
                    m_flippyWrist.setGoal(FlippyWristConstants.kIntakeHalf);
                }else {
                    m_flippyWrist.setGoal(FlippyWristConstants.kIntakeGround);    
                }
                setWrist = true;
            }
            
        }else{
            if(GlobalVariables.m_targetCoralLevel > 2 & !setElevator){
                m_elevator.setGoal(ElevatorConstants.kIntakeGround);
                setElevator = true;
      
            }else if(!setElevator  && m_spinnyWrist.inPosition(SpinnyWristConstants.kIntake) &&  m_flippyWrist.inPosition(FlippyWristConstants.kIntakeGround)){
                m_elevator.setGoal(ElevatorConstants.kIntakeGround);
                setElevator = true;
            }

            if(!setSpinnyWrist){
                m_spinnyWrist.setGoal(SpinnyWristConstants.kIntake);
                setSpinnyWrist = true;
            }

            if (!(m_elevator.getPosition() > ElevatorConstants.kDeployL4)){
                if(!setWrist){
                if (GlobalVariables.m_haveAlgae){
                    m_flippyWrist.setGoal(FlippyWristConstants.kIntakeHalf);
                }else {
                    m_flippyWrist.setGoal(FlippyWristConstants.kIntakeGround);    
                }
                    setWrist = true;
                }
            }else{
                if (!setElevator){
                    if (GlobalVariables.m_haveAlgae){
                        m_elevator.setGoal(ElevatorConstants.kIntakeHalf);
                    }else {
                        m_elevator.setGoal(ElevatorConstants.kIntakeGround);    
                    }
                    setElevator = true;
                }
            }

            if(setElevator && m_elevator.inPosition(ElevatorConstants.kIntakeGround) && m_elevator.inPosition() && m_flippyWrist.inPosition(FlippyWristConstants.kIntakeGround) && !setPivot){
                if (GlobalVariables.m_haveAlgae){
                    m_pivot.setGoal(PivotConstants.kIntakeHalf);
                }else {
                    m_pivot.setGoal(PivotConstants.kIntakeGround);    
                }
                setPivot = true;
            }
        }


        

    }

    @Override
    public boolean isFinished(){
        //finish once all parts are in the correct spot
        return GlobalVariables.m_groundHasCoral;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.setCurrentLimit(30);
        m_groundIntake.setVoltage(GroundIntakeConstants.kHolding);

        if(interrupted){
            m_groundPivot.setGoal(GroundPivotConstants.kTransfer);
            GlobalVariables.m_groundHasCoral = false;
            return;
        }else{
            m_groundPivot.setGoal(GroundPivotConstants.kTransfer);
            setGroundPivot = true;
            m_groundIntake.setVoltage(GroundIntakeConstants.kHolding);
            GlobalVariables.m_groundHasCoral = true;
            m_variables.setRobotState(RobotState.HOLDING);
        }
    }
}
