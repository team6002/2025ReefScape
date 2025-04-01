package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_ScoreLevelOne extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final SUB_Pivot m_pivot;
    private final SUB_Intake m_intake;
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_GroundIntake m_groundIntake;
    private final GlobalVariables m_variables;

    private boolean setPivot;
    private boolean setWrist;
    private boolean setElevator;
    private boolean setGroundPivot;
    private boolean setFlippyWrist;

    public CMD_ScoreLevelOne(SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, SUB_Pivot p_pivot,
        SUB_Intake p_intake, SUB_GroundPivot p_groundPivot, SUB_GroundIntake p_groundIntake, GlobalVariables p_variables){

        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_spinnyWrist = p_spinnyWrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_groundPivot = p_groundPivot;
        m_groundIntake = p_groundIntake;
        m_variables = p_variables;  

        addRequirements(m_elevator, m_flippyWrist, m_spinnyWrist, m_pivot, m_intake, m_groundPivot, m_groundIntake);
    }

    @Override
    public void initialize(){
        m_variables.setRobotState(RobotState.DEPLOY);
        m_groundPivot.setGoal(Math.toRadians(5));

        setPivot = false;
        setElevator = false;
        setWrist = false;
        setFlippyWrist = false;
        setGroundPivot = false;
    }

    @Override
    public void execute(){
        if(!setGroundPivot){
            m_groundPivot.setGoal(5);
            m_groundIntake.setVoltage(-1);
            setGroundPivot = true;
        }

        if(!setElevator){
            m_elevator.setGoal(ElevatorConstants.kHome);
            setElevator = true;
        }

        if(!setWrist){
            m_flippyWrist.setGoal(Math.toRadians(-120));
            m_spinnyWrist.setGoal(SpinnyWristConstants.kIntake);
            setWrist = true;
        }

        if(setWrist && m_flippyWrist.inPosition() &! setPivot){
            m_pivot.setGoal(Math.toRadians(PivotConstants.kIntakeGround));
            setPivot = true;
        }

        if(setElevator && setGroundPivot && setWrist && setPivot && m_pivot.inPosition() && m_elevator.inPosition() 
            && m_groundPivot.inPosition() && m_flippyWrist.inPosition() && m_spinnyWrist.inPosition() &! setFlippyWrist){
                m_flippyWrist.setGoal(FlippyWristConstants.kDeployL1);
                setFlippyWrist = true;
        }
    }

    @Override
    public boolean isFinished(){
        return setElevator && setGroundPivot && setWrist && setPivot && m_pivot.inPosition() && m_elevator.inPosition() 
        && m_groundPivot.inPosition() && m_flippyWrist.inPosition() && m_spinnyWrist.inPosition() && setFlippyWrist;
    }

    // @Override
    // public void end(boolean interrutped){
        
    // }
}
