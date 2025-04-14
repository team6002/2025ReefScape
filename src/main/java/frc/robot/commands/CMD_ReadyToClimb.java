package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Winch.SUB_Winch;

public class CMD_ReadyToClimb extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_Intake m_intake;
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_Winch m_winch;
    private final SUB_GroundIntake m_groundIntake;

    private boolean setPivot;
    private boolean setWrist;
    private boolean setElevator;
    private boolean setGroundPivot;
    private boolean setClimb;

    private Timer groundPivotTimer = new Timer();

    public CMD_ReadyToClimb(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_Intake p_intake, 
        SUB_GroundPivot p_groundPivot, SUB_Winch p_winch, SUB_GroundIntake p_groundIntake){
            
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_groundPivot = p_groundPivot;
        m_winch = p_winch;
        m_groundIntake = p_groundIntake;

        addRequirements(m_pivot, m_elevator, m_flippyWrist, m_intake, m_groundPivot, m_groundIntake);
    }

    @Override
    public void initialize(){
        setPivot = false;
        setElevator = false;
        setWrist = false;
        setGroundPivot = false;
        setClimb = false;

        m_intake.setVoltage(IntakeConstants.kOff);
        m_groundIntake.setVoltage(GroundIntakeConstants.kReverse);
        
        groundPivotTimer.reset();
        groundPivotTimer.stop();
    }

    @Override
    public void execute(){
        if(!setPivot){
            m_pivot.setGoal(PivotConstants.kClimb);
            setPivot = true;
        }

        if(setPivot && m_pivot.inPosition() && !setGroundPivot){
            m_groundPivot.setGoal(GroundPivotConstants.kClimb);
            setGroundPivot = true;
            groundPivotTimer.start();
        }

        if(setGroundPivot && setPivot && m_pivot.inPosition() && m_groundPivot.inPosition() && !setElevator || (groundPivotTimer.get() > 2 && !setWrist)){
            m_elevator.setGoal(ElevatorConstants.kClimb);
            setElevator = true;
        }

        if(setGroundPivot && setPivot && m_pivot.inPosition() && m_groundPivot.inPosition() && !setWrist || (groundPivotTimer.get() > 2 && !setWrist)){
            m_flippyWrist.setGoal(FlippyWristConstants.kClimb);
            setWrist = true;
        }

        if(setElevator && setPivot && setWrist && setGroundPivot && m_elevator.inPosition()
            && m_flippyWrist.inPosition() && m_pivot.inPosition() && !setClimb){
            
            m_winch.setReference(WinchConstants.kReadyClimb);
            setClimb = true;
        }
    }

    @Override
    public boolean isFinished(){
        //finish once all parts are in the correct spot
        return setElevator && setPivot && setWrist && setGroundPivot && m_elevator.inPosition()
        && m_flippyWrist.inPosition() && m_pivot.inPosition() && setClimb;
    }

    @Override
    public void end(boolean interrupted){
        m_groundIntake.setVoltage(GroundIntakeConstants.kClimb);
    }
}