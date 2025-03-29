package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.GlobalVariables;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;
import frc.robot.Constants.IntakeConstants;
import frc.GlobalVariables.RobotState;

public class CMD_Score extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_FlippyWrist m_flippyWrist;
    private final SUB_SpinnyWrist m_spinnyWrist;
    private final SUB_Pivot m_pivot;
    private final SUB_Intake m_intake;
    private final SUB_GroundPivot m_groundPivot;
    private final SUB_GroundIntake m_groundIntake;
    private final GlobalVariables m_variables;

    public CMD_Score(SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, SUB_Pivot p_pivot,
        SUB_Intake p_intake, SUB_GroundPivot p_groundPivot, SUB_GroundIntake p_groundIntake, GlobalVariables p_variables){

        m_elevator = p_elevator;
        m_flippyWrist = p_flippyWrist;
        m_spinnyWrist = p_spinnyWrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_groundPivot = p_groundPivot;
        m_groundIntake = p_groundIntake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch (m_variables.getRobotState()) {
            //if home or ready, go to ready to intake
            case HOME:
            case READY:
                GlobalVariables.m_haveAlgae = false;
                m_intake.setVoltage(IntakeConstants.kOff);
                new ConditionalCommand(
                    new CMD_ReadyToIntake(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_intake, m_variables)         
                    ,new CMD_ReadyToIntakeFromGround(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_intake, m_groundPivot, m_groundIntake, m_variables)
                    ,()-> GlobalVariables.m_intakeFromStation
                ).schedule();
                break;
            //if ready to intake and do not have algae go to deploy, if we do have an algae, go to ready
            case READY_TO_INTAKE:
                if(!GlobalVariables.m_haveAlgae) new CMD_ReadyToDeploy(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_variables).schedule();
                else new CMD_Ready(m_pivot, m_elevator, m_flippyWrist, m_intake, m_variables).schedule();
                break;
            //if ready to deploy, shoot
            case READY_TO_DEPLOY:
                if(GlobalVariables.m_groundHasCoral){
                    new CMD_GroundIntake(m_groundPivot, m_groundIntake).schedule();
                }else{
                    new CMD_Deploy(m_intake, m_flippyWrist, m_variables).schedule();
                }
                break;
            //if we just shot, go back to intaking if no algae, and ready if algae
            case DEPLOY:
                if(!GlobalVariables.m_haveAlgae) new ConditionalCommand(
                    new CMD_ReadyToIntake(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_intake, m_variables)         
                    ,new CMD_ReadyToIntakeFromGround(m_pivot, m_elevator, m_flippyWrist, m_spinnyWrist, m_intake, m_groundPivot, m_groundIntake, m_variables)
                    ,()-> GlobalVariables.m_intakeFromStation
                ).schedule();
                else new CMD_Ready(m_pivot, m_elevator, m_flippyWrist, m_intake, m_variables).schedule();
                break;
            //if we just grabbed an algae of the reef, ready to deploy alt sequence
            case ALGAE_LEVEL_2:
            case ALGAE_LEVEL_3:
                new CMD_AlgaeReadyToDeploy(m_pivot, m_elevator, m_flippyWrist, m_variables).schedule();
                break;
            //if ready to score in barge/proccesor, spit out algae, and set state to ready so the next RB press goes to ready to intake
            case BARGE:
            case PROCESSOR:
                new InstantCommand(()->  m_intake.setVoltage(IntakeConstants.kReverse)).schedule();
                new InstantCommand(()-> GlobalVariables.m_haveAlgae = false).schedule();
                new InstantCommand(()-> m_variables.setRobotState(RobotState.READY)).schedule();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
