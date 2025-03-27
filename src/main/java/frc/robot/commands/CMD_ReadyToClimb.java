package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.robot.Constants.*;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Winch.SUB_Winch;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToClimb extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Intake m_intake;
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_Winch m_winch;

    private boolean setPivot;
    private boolean setWrist;
    private boolean setElevator;
    private boolean setGroundPivot;
    private boolean setClimb;

    public CMD_ReadyToClimb(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Intake p_intake, 
        SUB_GroundPivot p_groundPivot, SUB_Winch p_winch){
            
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_intake = p_intake;
        m_groundPivot = p_groundPivot;
        m_winch = p_winch;

        addRequirements(m_pivot, m_elevator, m_wrist, m_intake, m_groundPivot);
    }

    @Override
    public void initialize(){
        setPivot = false;
        setElevator = false;
        setWrist = false;
        setGroundPivot = false;
        setClimb = false;

        if(GlobalVariables.m_haveCoral){
            m_intake.setVoltage(IntakeConstants.kHolding);
        }else{
            m_intake.setVoltage(IntakeConstants.kOff);
        }
    }

    @Override
    public void execute(){
        if(!setPivot){
            m_pivot.setGoal(PivotConstants.kClimb);
            setPivot = true;
        }

        if(setPivot && m_pivot.inPosition() &! setGroundPivot){
            m_groundPivot.setGoal(GroundPivotConstants.kHome);
            setGroundPivot = true;
        }

        if(setGroundPivot && setPivot && m_pivot.inPosition() && m_groundPivot.inPosition() &! setElevator){
            m_elevator.setGoal(ElevatorConstants.kHome);
            setElevator = true;
        }

        if(setGroundPivot && setPivot && m_pivot.inPosition() && m_groundPivot.inPosition() &! setWrist){
            m_wrist.setGoal(WristConstants.kClimb);
            setWrist = true;
        }

        if(setElevator && setPivot && setWrist && setGroundPivot && m_elevator.inPosition() && m_groundPivot.inPosition()
            && m_wrist.inPosition() && m_pivot.inPosition() &! setClimb){
            
            m_winch.setReference(WinchConstants.kReadyClimb);
            setClimb = true;
        }
    }

    @Override
    public boolean isFinished(){
        //finish once all parts are in the correct spot
        return setElevator && setPivot && setWrist && setGroundPivot && m_elevator.inPosition() && m_groundPivot.inPosition()
        && m_wrist.inPosition() && m_pivot.inPosition() && setClimb;
    }
}