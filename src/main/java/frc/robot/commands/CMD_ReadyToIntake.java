package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntake extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;
    
    private boolean elevatorInPosition;
    private boolean pivotInPosition;
    private boolean wristInPosition;

    private boolean elevatorInMotion;
    private boolean pivotInMotion;
    private boolean wristInMotion;

    private final Timer m_intakeTimer = new Timer();

    private boolean haveCoral;

    public CMD_ReadyToIntake(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Intake p_intake, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_intake = p_intake;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_wrist, m_intake);
    }

    @Override
    public void initialize(){
        pivotInPosition = false;
        elevatorInPosition = false;
        wristInPosition = false;

        pivotInMotion = false;
        elevatorInMotion = false;
        wristInMotion = false;

        haveCoral = false;

        m_intake.setVoltage(IntakeConstants.kIntake);

        m_intakeTimer.reset();
        m_intakeTimer.stop();
    }

    @Override
    public void execute(){
        //if elevator is lower than intake pos, can move pivot right away
        if(m_elevator.BelowPosition(ElevatorConstants.kIntake, false)){
            //move pivot to ready to intake if not there already
            if(!m_pivot.inPosition(PivotConstants.kReadyIntake)){
                if(!pivotInMotion) m_pivot.setGoal(PivotConstants.kReadyIntake);
                pivotInMotion = true;
            }else{
                pivotInMotion = false;
                pivotInPosition = true;
            }
            //move elevator if not within tolerance of intake
            if(!m_elevator.inPosition(ElevatorConstants.kIntake)){
                if(!elevatorInMotion) m_elevator.setGoal(ElevatorConstants.kIntake);
                elevatorInMotion = true;
                //if elevator is in correct spot, begin moving wrist
            }else if(!m_wrist.inPosition(WristConstants.kIntake)){
                if(!wristInMotion) m_wrist.setGoal(WristConstants.kIntake);
                wristInMotion = true;
                elevatorInMotion = false;
                elevatorInPosition = true;
            }else{
                wristInMotion = false;
                wristInPosition = true;
            }
            //if elevator is too high, we must move it first, this also means we can move the wrist as we move the elevator
        }else{
            //if elevator not ready to intake, move it
            if(!m_elevator.inPosition(ElevatorConstants.kIntake)){
                if(!elevatorInMotion) m_elevator.setGoal(ElevatorConstants.kIntake);
                elevatorInMotion = true;
            }else{
                elevatorInMotion = false;
                elevatorInPosition = true;
            }
            
            //if wrist not ready to intake, move it
            if(!m_wrist.inPosition(WristConstants.kIntake)){
                if(pivotInPosition){
                    if(!wristInMotion) m_wrist.setGoal(WristConstants.kIntake);
                    wristInMotion = true;
                }
            }else{
                wristInPosition = true;
                wristInMotion = false;
            }

            //if wrist and elevator are in the correct spot, and pivot is not, move the pivot
            if(wristInPosition && elevatorInPosition && !m_pivot.inPosition(PivotConstants.kReadyIntake)){
                if(!pivotInMotion) m_pivot.setGoal(PivotConstants.kReadyIntake);
                pivotInMotion = true;
            }else{
                pivotInMotion = false;
                pivotInPosition = true;
            }
        }

        if(pivotInPosition && elevatorInPosition && wristInPosition){
            m_variables.setRobotState(RobotState.READY_TO_INTAKE);
        }

        if(elevatorInPosition && pivotInPosition  && wristInPosition){
            if(m_intake.getCurrent() > 18){
                m_intakeTimer.start();
            }else{
                m_intakeTimer.reset();
            }
    
            if(m_intakeTimer.get() > 0.1){
                haveCoral = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        //finish once all parts are in the correct spot
        return pivotInPosition && elevatorInPosition && wristInPosition && haveCoral;
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted){
            GlobalVariables.m_haveCoral = false;
            m_intake.setVoltage(IntakeConstants.kOff);
        }else{
            GlobalVariables.m_haveCoral = true;
            m_intake.setVoltage(IntakeConstants.kHolding);
            new CMD_ReadyToDeploy(m_pivot, m_elevator, m_wrist, m_variables).schedule();
        }
    }
}
