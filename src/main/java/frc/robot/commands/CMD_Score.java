package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Score extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_Intake m_intake;
    private final GlobalVariables m_variables;

    public CMD_Score(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Intake p_intake, GlobalVariables p_variables){
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch (m_variables.getRobotState()) {
            case HOME:
                new CMD_ReadyToIntake(m_pivot, m_elevator, m_wrist, m_intake, m_variables).schedule();
                break;
            case READY_TO_INTAKE:
                new CMD_ReadyToDeploy(m_pivot, m_elevator, m_wrist, m_variables).schedule();
                break;
            case READY_TO_DEPLOY:
                new CMD_Deploy(m_intake, m_wrist, m_variables).schedule();
                break;
            case DEPLOY:
                if(GlobalVariables.m_targetCoralLevel == 2){
                    new CMD_ChangeLevel(m_pivot, m_elevator, m_wrist, m_variables, 2).schedule();
                }else if(GlobalVariables.m_targetCoralLevel == 3){
                    new CMD_ChangeLevel(m_pivot, m_elevator, m_wrist, m_variables, 3).schedule();
                }else if(GlobalVariables.m_targetCoralLevel == 4){
                    new CMD_ChangeLevel(m_pivot, m_elevator, m_wrist, m_variables, 4).schedule();
                }
                break;
            case READY_TO_SCORE:
                new CMD_Deploy(m_intake, m_wrist, m_variables)
                .andThen(new WaitCommand(.5))
                .andThen(new CMD_ReadyToIntake(m_pivot, m_elevator, m_wrist, m_intake, m_variables)).schedule();
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
