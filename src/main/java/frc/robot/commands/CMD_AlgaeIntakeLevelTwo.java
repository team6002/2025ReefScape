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

public class CMD_AlgaeIntakeLevelTwo extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;

    private boolean setElevator1;
    private boolean setElevator2;
    private boolean setWrist;
    private boolean setPivot;

    private final Timer m_triggerTimer = new Timer();
    private final Timer m_SUCTimer = new Timer();
    private boolean isFinished = false;


    public CMD_AlgaeIntakeLevelTwo(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_Intake p_intake, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_intake = p_intake;
        m_variables = p_variables;

        addRequirements(m_pivot, m_elevator, m_flippyWrist);
    }

    @Override
    public void initialize(){
        setElevator1 = false;
        setElevator2 = false;
        setPivot = false;
        setWrist = false;

        m_triggerTimer.reset();
        m_triggerTimer.start();
        m_SUCTimer.reset();
        m_SUCTimer.stop();
        isFinished = false;

        m_intake.setCurrentLimit(40);
        m_intake.setVoltage(IntakeConstants.kAlgaeIntake);;
    }


    @Override
    public void execute(){
        if (!setElevator1){
            setElevator1 = true;
            m_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgael2Up);
        }

        if(!setWrist && setElevator1 && m_elevator.inPosition()){
            setWrist = true;
            m_flippyWrist.setGoal(FlippyWristConstants.kReadyIntakeAlgae);
        }

        if(setWrist && !setElevator2){
            setElevator2 = true;
            m_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgael2Down);
        }

        if(setWrist && setElevator2 && m_elevator.inPosition() && !setPivot){
            setPivot = true;
            m_pivot.setGoal(PivotConstants.kReadyIntakeAlgae);
        }

        if(setWrist && setElevator2 && setPivot && m_elevator.inPosition() && m_pivot.inPosition()){
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
        if(interrupted){m_intake.setVoltage(IntakeConstants.kOff); return;}
        m_pivot.setGoal(PivotConstants.kAlgaeLvl2Hold);
        m_intake.setVoltage(IntakeConstants.kAlgaeHolding);
    }
}
