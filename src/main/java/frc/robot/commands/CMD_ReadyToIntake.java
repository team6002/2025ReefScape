package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_ReadyToIntake extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;

    private final Timer m_intakeTimer = new Timer();

    private boolean haveCoral;
    private boolean setPivot;
    private boolean setWrist;
    private boolean setElevator;

    public CMD_ReadyToIntake(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, 
        SUB_Intake p_intake, GlobalVariables p_variables){
            
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_spinnyWrist = p_spinnyWrist;
        m_intake = p_intake;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_intake);
    }

    @Override
    public void initialize(){
        haveCoral = false;
        setPivot = false;
        setElevator = false;
        setWrist = false;

        m_intake.setVoltage(IntakeConstants.kIntake);
        m_intake.setConveyorVoltage(IntakeConstants.kConveyorOff);

        m_intakeTimer.reset();
        m_intakeTimer.stop();
    }

    @Override
    public void execute(){
        if(m_pivot.getPosition() <= PivotConstants.kIntake){
            if(!setPivot){
                m_pivot.setGoal(PivotConstants.kIntake);
                setPivot = true;
            }

            if(m_pivot.inPosition(PivotConstants.kIntake) &! setElevator){
                m_elevator.setGoal(ElevatorConstants.kIntake);
                setElevator = true;
            }

            if(m_pivot.inPosition(PivotConstants.kIntake) && m_elevator.inPosition(ElevatorConstants.kIntake) &! setWrist){
                m_flippyWrist.setGoal(FlippyWristConstants.kIntake);
                m_spinnyWrist.setGoal(SpinnyWristConstants.kIntake);
                setWrist = true;
            }
        }else{
            if(!setElevator){
                m_elevator.setGoal(ElevatorConstants.kIntake);
                setElevator = true;
            }

            if(!setWrist && setElevator && m_elevator.inPosition(ElevatorConstants.kIntake)){
                m_flippyWrist.setGoal(FlippyWristConstants.kIntake);
                m_spinnyWrist.setGoal(SpinnyWristConstants.kIntake);
                setWrist = true;
            }

            if(setElevator && m_elevator.inPosition(ElevatorConstants.kIntake) && m_flippyWrist.inPosition(FlippyWristConstants.kIntake) &! setPivot){
                m_pivot.setGoal(PivotConstants.kIntake);
                setPivot = true;
            }
        }


        if(m_elevator.inPosition(ElevatorConstants.kIntake) && m_flippyWrist.inPosition(FlippyWristConstants.kIntake) && m_pivot.inPosition(PivotConstants.kIntake)){
            m_variables.setRobotState(RobotState.READY_TO_INTAKE);
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
        return haveCoral;
    }

    @Override
    public void end(boolean interrupted){
        m_spinnyWrist.setGoal(SpinnyWristConstants.kHome);
        m_intake.setVoltage(IntakeConstants.kOff);

        if(interrupted){
            GlobalVariables.m_haveCoral = false;
            return;
        }

        new CMD_ReadyToDeploy(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_variables).schedule();
    }
}
