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

public class CMD_ReadyToDeploy extends Command{
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Intake m_intake;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final SUB_GroundPivot m_groundPivot;
    private final GlobalVariables m_variables;

    private boolean setElevator;
    private boolean setPivot;
    private boolean setFlippyWrist;
    private boolean setSpinnyWrist;
    private boolean setGroundPivot;
    private boolean regriper = false;
    private Timer regripTimer = new Timer();

    public CMD_ReadyToDeploy(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist, 
        SUB_SpinnyWrist p_spinnyWrist, SUB_GroundPivot p_groundPivot, GlobalVariables p_variables){
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_intake = p_intake;
        m_flippyWrist = p_flippyWrist;
        m_spinnyWrist = p_spinnyWrist;
        m_groundPivot = p_groundPivot;
        m_variables = p_variables;
        addRequirements(m_pivot, m_elevator, m_intake, m_flippyWrist, m_spinnyWrist, m_groundPivot);
    }

    @Override
    public void initialize(){
        setPivot = false;
        setElevator = false;
        setFlippyWrist = false; 
        setSpinnyWrist = false;
        regriper = false;
    
        m_intake.setVoltage(IntakeConstants.kOff);

        if(GlobalVariables.m_targetCoralLevel >= 3){
            GlobalVariables.m_targetCoralLevel = 3;
        }else{
            GlobalVariables.m_targetCoralLevel = 2;
        }

        m_variables.setRobotState(RobotState.READY_TO_DEPLOY);
        
        regripTimer.reset();
        regripTimer.start();;
    }

    @Override
    public void execute(){
        
        
        if (!regriper && regripTimer.get() > .1){
            m_intake.setVoltage(IntakeConstants.kHolding);
        }
        //if level 3/4 are selected go to level 3 automatically(can change level manually later)
        if(GlobalVariables.m_targetCoralLevel >= 3){
            if(GlobalVariables.m_placeFront){
                if(!setPivot){
                    m_pivot.setGoal(PivotConstants.kDeployFrontL3);
                    setPivot = true;
                }
        
                if(!m_elevator.inPosition(ElevatorConstants.kDeployFrontL3) && !setElevator && setPivot && m_pivot.inPosition()){
                    m_elevator.setGoal(ElevatorConstants.kDeployFrontL3);
                    setElevator = true;
                }
        
                if(!m_flippyWrist.inPosition(FlippyWristConstants.kDeployFrontL3) && !setFlippyWrist && setPivot && m_pivot.inPosition()){
                    m_flippyWrist.setGoal(FlippyWristConstants.kDeployFrontL3);
                    setFlippyWrist = true;
                }
        
                if(m_flippyWrist.inPosition(FlippyWristConstants.kDeployFrontL3) && !setSpinnyWrist && setPivot && m_pivot.inPosition()){
                    m_spinnyWrist.setGoal(SpinnyWristConstants.kDeployFront);
                    setSpinnyWrist = true;
                }
            }else{
                if(!m_pivot.inPosition(PivotConstants.kDeployL3) && !setPivot){
                    m_pivot.setGoal(PivotConstants.kDeployL3);
                    setPivot = true;
                }

                if(!m_elevator.inPosition(ElevatorConstants.kDeployL3) && !setElevator){
                    m_elevator.setGoal(ElevatorConstants.kDeployL3);
                    setElevator = true;
                }

                if(!m_flippyWrist.inPosition(FlippyWristConstants.kDeployL3) && !setFlippyWrist){
                    m_flippyWrist.setGoal(FlippyWristConstants.kDeployL3);
                    setFlippyWrist = true;
                }

                if(m_elevator.inPosition(ElevatorConstants.kDeployL3) && !setSpinnyWrist){
                    m_spinnyWrist.setGoal(SpinnyWristConstants.kHome);
                    setSpinnyWrist = true;
                }
            }
            //if not level 3/4, go to l2 automatically
        }else{
            if(GlobalVariables.m_placeFront){
                if(!setGroundPivot){
                    m_groundPivot.setGoal(GroundPivotConstants.kStart);
                    m_pivot.setGoal(PivotConstants.kChangeLevelTwo);
                    setGroundPivot = true;
                }

                if(setGroundPivot && m_groundPivot.inPosition() &! setPivot){
                    m_pivot.setGoal(PivotConstants.kDeployFrontL2);
                    setPivot = true;
                }

                if(setPivot && m_pivot.inPosition() &! setElevator){
                    m_elevator.setGoal(ElevatorConstants.kDeployFrontL2);
                    setElevator = true;
                }

                if(setPivot && m_pivot.inPosition() &! setFlippyWrist){
                    m_flippyWrist.setGoal(FlippyWristConstants.kDeployFrontL2);
                    setFlippyWrist = true;
                }

                if(setPivot && m_pivot.inPosition() &! setSpinnyWrist){
                    m_spinnyWrist.setGoal(SpinnyWristConstants.kDeployFront);
                    setSpinnyWrist = true;
                }
            }else{
                if(!m_pivot.inPosition(PivotConstants.kDeployL2) && !setPivot){
                    m_pivot.setGoal(PivotConstants.kDeployL2);
                    setPivot = true;
                }

                if(!m_elevator.inPosition(ElevatorConstants.kDeployL2) && !setElevator){
                    m_elevator.setGoal(ElevatorConstants.kDeployL2);
                    setElevator = true;
                }

                if(!m_flippyWrist.inPosition(FlippyWristConstants.kDeployL2) && !setFlippyWrist){
                    m_flippyWrist.setGoal(FlippyWristConstants.kDeployL2);
                    setFlippyWrist = true;
                }

                if(m_elevator.inPosition(ElevatorConstants.kDeployL2) && !setSpinnyWrist){
                    m_spinnyWrist.setGoal(SpinnyWristConstants.kHome);
                    setSpinnyWrist = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished(){
        return setPivot && setElevator && setFlippyWrist && setSpinnyWrist && m_elevator.inPosition() && m_pivot.inPosition() && m_flippyWrist.inPosition() && m_spinnyWrist.inPosition();
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) return;
    }
}
