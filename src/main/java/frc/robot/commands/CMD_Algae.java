package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_Algae extends Command{
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_Elevator m_elevator;
    private final SUB_Algae m_algae;
    private final SUB_CoralHolder m_intake;
    private final GlobalVariables m_variables;
    private boolean m_deployingAlgae;
    private AlgaeTarget m_lastTarget;
    public CMD_Algae(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae, 
        SUB_CoralHolder p_intake, GlobalVariables p_variales){
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_algae = p_algae;
        m_intake = p_intake;
        m_variables = p_variales;
        addRequirements(m_algae);
    }

    @Override
    public void initialize(){
        // allows for switching from level 2 -> 3 as well as manually stopping intaking
        if(GlobalVariables.m_intakingAlgae && m_variables.getAlgaeTarget() != m_lastTarget){
            m_deployingAlgae = false;
        }
        
        if(GlobalVariables.m_intakingAlgae && m_variables.getAlgaeTarget() == m_lastTarget){
            m_variables.setRobotState(RobotState.HOME);
            GlobalVariables.m_haveAlgae = false;
            new CMD_Score(m_elevator, m_wrist, m_intake, m_pivot, m_algae, m_variables).schedule();
            return;
        }

        //if ready to deploy, shoot, and go back to intake coral
        if(m_deployingAlgae){
            m_deployingAlgae = false;
            GlobalVariables.m_haveAlgae = false;
            m_lastTarget = null;
            new SequentialCommandGroup(
                new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kReverse))
                ,new CMD_CheckCoral(m_intake)
                ,new WaitCommand(.5)
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kOff))
                ,new CMD_SetReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
            ).schedule();
            return;
        }

        switch (m_variables.getAlgaeTarget()) {
            case LEVEL_2:
                GlobalVariables.m_intakingAlgae = true;
                m_deployingAlgae = false;
                m_lastTarget = AlgaeTarget.LEVEL_2;
                new CMD_AlgaeLevelTwo(m_wrist, m_pivot, m_elevator, m_algae, m_intake, m_variables).schedule();
                break;
            case LEVEL_3:
                GlobalVariables.m_intakingAlgae = true;
                m_deployingAlgae = false;
                m_lastTarget = AlgaeTarget.LEVEL_3;
                new ConditionalCommand(
                    new InstantCommand(()-> GlobalVariables.lvl3AlgaeException = true).andThen(new InstantCommand(()-> GlobalVariables.m_targetCoralLevel = 2))
                    ,new CMD_AlgaeLevelThree(m_intake, m_wrist, m_algae, m_elevator, m_variables, m_pivot)
                    ,()-> GlobalVariables.m_algaeExceptionMode
                ).schedule();   
                break;
            case GROUND:
                GlobalVariables.m_intakingAlgae = true;
                m_deployingAlgae = false;
                m_lastTarget = AlgaeTarget.GROUND;
                new CMD_SetReadyIntakeAlgaeGround(m_pivot, m_elevator, m_wrist, m_algae, m_intake, m_variables).schedule();
                break;
            case BARGE:
                m_deployingAlgae = true;
                GlobalVariables.m_intakingAlgae = false;
                m_lastTarget = AlgaeTarget.BARGE;
                new CMD_ReadyToDeployBarge(m_wrist, m_pivot, m_elevator).schedule();
                break;
            case PROCESSOR:
                m_deployingAlgae = true;
                GlobalVariables.m_intakingAlgae = false;
                m_lastTarget = AlgaeTarget.PROCESSOR;
                new CMD_ReadyToDeployProcessor(m_elevator, m_wrist, m_pivot).schedule();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        Logger.recordOutput("lastAlgaeTarget", m_lastTarget);
    }
}
