package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.GlobalVariables;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.Constants.AlgaeConstants;
import frc.GlobalVariables.RobotState;

public class CMD_Score extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_Intake m_intake;
    private final SUB_Algae m_algae;
    private final GlobalVariables m_variables;

    public CMD_Score(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Intake p_intake, SUB_Algae p_algae,
        GlobalVariables p_variables){

        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_variables = p_variables;
        m_algae = p_algae;
    }

    @Override
    public void initialize(){
        switch (m_variables.getRobotState()) {
            //if home or ready, go to ready to intake
            case HOME:
            case READY:
                new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kOff)).schedule();
                new CMD_ReadyToIntake(m_pivot, m_elevator, m_wrist, m_intake, m_variables).schedule();
                break;
            //if ready to intake and do not have algae go to deploy, if we do have an algae, go to ready
            case READY_TO_INTAKE:
                if(!GlobalVariables.m_haveAlgae) new CMD_ReadyToDeploy(m_pivot, m_elevator, m_wrist, m_variables).schedule();
                else new CMD_Ready(m_pivot, m_elevator, m_wrist, m_intake, m_variables).schedule();
                break;
            //if ready to deploy, shoot
            case READY_TO_DEPLOY:
                new CMD_Deploy(m_intake, m_wrist, m_variables).schedule();
                break;
            //if we just shot, go back to intaking if no algae, and ready if algae
            case DEPLOY:
                if(!GlobalVariables.m_haveAlgae) new CMD_ReadyToIntake(m_pivot, m_elevator, m_wrist, m_intake, m_variables).schedule();
                else new CMD_Ready(m_pivot, m_elevator, m_wrist, m_intake, m_variables).schedule();
                break;
            //if we just grabbed an algae of the reef, ready to deploy alt sequence
            case ALGAE_LEVEL_2:
            case ALGAE_LEVEL_3:
                new CMD_AlgaeReadyToDeploy(m_pivot, m_elevator, m_wrist, m_variables).schedule();
                break;
            //if ready to score in barge/proccesor, spit out algae, and set state to ready so the next RB press goes to ready to intake
            case BARGE:
            case PROCESSOR:
                new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kReverse)).schedule();
                new InstantCommand(()-> GlobalVariables.m_haveAlgae = false).schedule();
                new InstantCommand(()-> m_variables.setRobotState(RobotState.READY)).schedule();
                break;
            //allows for cancellation of ground algae intake, and puts robot back in ready, either to try again or go grab a coral
            case GROUND:
                new CMD_Ready(m_pivot, m_elevator, m_wrist, m_intake, m_variables).schedule();
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
