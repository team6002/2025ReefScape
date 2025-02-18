package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.Mode;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_Score extends Command{
    SUB_Elevator m_elevator;
    SUB_Wrist m_wrist;
    SUB_CoralHolder m_intake;
    SUB_Pivot m_pivot;
    SUB_Algae m_algae;
    GlobalVariables m_variables;
    public CMD_Score(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_CoralHolder p_intake, SUB_Pivot p_pivot, 
                     SUB_Algae p_algae, GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_intake = p_intake;
        m_pivot = p_pivot;
        m_algae = p_algae;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch(m_variables.getRobotState()){
           case HOME:
                //ready, no object
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,ready()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY))
                ).schedule();
                break;
            case READY:
                //ready to intake
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_INTAKE))
                    ,new CMD_ReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_INTAKE))
                    ,new CMD_IntakeStow(m_intake)
                    ,ready()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_STOWED))
                    ,new InstantCommand(()-> GlobalVariables.m_haveCoral = true)
                ).schedule();
                break;
            case READY_TO_INTAKE:
                //ready with an object to score
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,ready()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_STOWED))
                    ,new InstantCommand(()-> GlobalVariables.m_haveCoral = true)
                ).schedule();
                break;
           case READY_STOWED:
                //ready to deploy
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                    ,readyToDeploy()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_DEPLOY))
                ).schedule();
                break;
            case READY_TO_DEPLOY:
                //deploy/score
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                    ,deployCommand()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.DEPLOY))
                    ,new InstantCommand(()-> GlobalVariables.m_haveCoral = false)
                ).schedule();
                break;
            case DEPLOY:
                //ready, no object
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,readyHome()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY))
                ).schedule();
                break;
            default:
                break;
        }
    }

    private ConditionalCommand readyHome(){
        return new ConditionalCommand(
            new CMD_ReadyHome(m_elevator, m_wrist, m_pivot, m_intake)
            ,new CMD_ReadyHomeDefensive(m_elevator, m_wrist, m_pivot, m_intake)
            ,()-> m_variables.isMode(Mode.OFFENSIVE)
        );
    }

    private ConditionalCommand ready(){
        return new ConditionalCommand(
            new CMD_Ready(m_elevator, m_wrist, m_pivot, m_intake)
            ,new CMD_ReadyDefensive(m_elevator, m_wrist, m_pivot, m_intake)
            ,()-> m_variables.isMode(Mode.OFFENSIVE)
        );
    }

    private SequentialCommandGroup readyToDeploy(){
        SequentialCommandGroup m_readyDeployCommand = new SequentialCommandGroup();
        switch (GlobalVariables.m_targetCoralLevel) {
            case 1:
                m_readyDeployCommand = new CMD_ReadyToDeployLevelOne(m_elevator, m_wrist, m_pivot);
                break;
            case 2:
                m_readyDeployCommand = new CMD_ReadyToDeployLevelTwo(m_elevator, m_wrist, m_pivot);
                break;
            case 3:
                m_readyDeployCommand = new CMD_ReadyToDeployLevelThree(m_elevator, m_wrist, m_pivot);
                break;
            case 4:
                m_readyDeployCommand = new CMD_ReadyToDeployLevelFour(m_elevator, m_wrist, m_pivot);
                break;
            default:
                break;
        }
        return m_readyDeployCommand; 
    }

    private SequentialCommandGroup deployCommand(){
        SequentialCommandGroup m_deployCommand = new SequentialCommandGroup();

        switch(GlobalVariables.m_targetCoralLevel){
            case 1:
                m_deployCommand = new CMD_DeployLevelOne(m_intake);
                break;
            case 2:
                m_deployCommand = new CMD_DeployLevelTwo(m_intake, m_wrist);
                break;
            case 3:
                m_deployCommand = new CMD_DeployLevelThree(m_intake, m_wrist);
                break;
            case 4:
                m_deployCommand = new CMD_DeployLevelFour(m_intake, m_wrist);
                break;
            default:
                break;
        }

        return m_deployCommand;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
