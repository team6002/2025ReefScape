package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.Mode;
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
       if(GlobalVariables.m_haveAlgae == false && (m_variables.isRobotState(RobotState.READY) || 
            m_variables.isRobotState(RobotState.READY_STOWED) || m_variables.isRobotState(RobotState.HOME))){
            new SequentialCommandGroup(
                new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_INTAKE))
                ,readyToIntake()
                ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_INTAKE))
                ,new CMD_AlgaeTrigger(m_algae)
                ,new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
                ,new CMD_ReadyAlgae(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kHolding))
                ,new InstantCommand(()-> setReady())
            ).schedule();
            return;
       }
       if(GlobalVariables.m_haveAlgae == false && m_variables.isRobotState(RobotState.READY_TO_INTAKE)){
            new SequentialCommandGroup(
                new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                ,new CMD_ReadyAlgae(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kHolding))
                ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
                ,new InstantCommand(()-> setReady())
            ).schedule();
            return;
       }
       if(GlobalVariables.m_haveAlgae &! m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            new SequentialCommandGroup(
                new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                ,readyToDeployAlgae()
                ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_DEPLOY))
            ).schedule();
            return;
       }
       if(GlobalVariables.m_haveAlgae && m_variables.isRobotState(RobotState.READY_TO_DEPLOY)){
            new SequentialCommandGroup(
                new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kReverse))
                ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = false)
                ,new InstantCommand(()-> m_variables.setRobotState(RobotState.DEPLOY))   
            ).schedule();
            return;
       }
       if(m_variables.isRobotState(RobotState.DEPLOY)){
        new SequentialCommandGroup(
            new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
            ,new ConditionalCommand(
                new CMD_Ready(m_elevator, m_wrist, m_pivot, m_intake), 
                new CMD_ReadyDefensive(m_elevator, m_wrist, m_pivot, m_intake),
                ()-> m_variables.isMode(Mode.OFFENSIVE))
            ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY))
            ,new InstantCommand(()-> m_algae.setReference(AlgaeConstants.kOff))
            ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = false)
        ).schedule();
        return;
       }
    }

    private SequentialCommandGroup readyToIntake(){
        SequentialCommandGroup m_readyIntakeCommand = new SequentialCommandGroup();
        switch (m_variables.getAlgaeTarget()) {
            case LEVEL_2:
                m_readyIntakeCommand = new CMD_ReadyToIntakeAlgaeTwo(m_wrist, m_pivot, m_elevator, m_algae);
                break;
            case LEVEL_3:
                m_readyIntakeCommand = new CMD_ReadyToIntakeAlgaeThree(m_wrist, m_pivot, m_elevator, m_algae);
                break;
            case GROUND:
                m_readyIntakeCommand = new CMD_AlgaeIntakeGround(m_wrist, m_pivot, m_elevator, m_algae);
                break;
            case CORAL:
                m_readyIntakeCommand = new CMD_AlgaeIntakeCoral(m_wrist, m_pivot, m_elevator, m_algae);
            default:
                break;
        }
        return m_readyIntakeCommand;
    }

    private SequentialCommandGroup readyToDeployAlgae(){
        SequentialCommandGroup m_readyToDeployCommand = new SequentialCommandGroup();

        switch (m_variables.getAlgaeTarget()) {
            case PROCESSOR:
                m_readyToDeployCommand = new CMD_ReadyDeployProcessor(m_elevator, m_wrist, m_pivot);
                break;
            case BARGE:
                m_readyToDeployCommand = new CMD_ReadyToDeployBarge(m_wrist, m_pivot, m_elevator);
                break;
            default:
                break;
        }
        
        return m_readyToDeployCommand;
    }

    private void setReady(){
        if(GlobalVariables.m_haveCoral){
            m_variables.setRobotState(RobotState.READY_STOWED);
        }else{
            m_variables.setRobotState(RobotState.READY);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
