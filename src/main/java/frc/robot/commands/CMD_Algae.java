package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
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
    // private boolean m_intakingAlgae = false;
    private boolean m_deployingAlgae = false;
    public CMD_Algae(SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_Algae p_algae, 
        SUB_CoralHolder p_intake, GlobalVariables p_variales){
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_elevator = p_elevator;
        m_algae = p_algae;
        m_intake = p_intake;
        m_variables = p_variales;
    }

    @Override
    public void initialize(){
        if(GlobalVariables.m_haveAlgae && m_deployingAlgae){
            new SequentialCommandGroup(
                new InstantCommand(()-> m_deployingAlgae = false)
                ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = false)
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kReverse))
                ,new WaitCommand(.5)
                ,new ConditionalCommand(
                    new CMD_ReadyAlgae(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                    ,new CMD_Ready(m_elevator, m_wrist, m_pivot, m_intake)
                    ,()-> m_variables.getAlgaeTarget() == AlgaeTarget.PROCESSOR
                )
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kOff))
            ).schedule();
            return;
        }

        if(GlobalVariables.m_haveAlgae){
            m_deployingAlgae = true;
            new ConditionalCommand(
                new CMD_ReadyToDeployBarge(m_wrist, m_pivot, m_elevator)
                ,new CMD_ReadyToDeployProcessor(m_elevator, m_wrist, m_pivot)
                ,()-> m_variables.getAlgaeTarget() == AlgaeTarget.BARGE).schedule();
            return;
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
