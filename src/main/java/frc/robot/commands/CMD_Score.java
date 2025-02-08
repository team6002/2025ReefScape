package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.Mode;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_Score extends Command{
    SUB_Elevator m_elevator;
    SUB_Wrist m_wrist;
    SUB_CoralHolder m_coralHolder;
    SUB_ElevatorPivot m_elevatorPivot;
    GlobalVariables m_variables;
    public CMD_Score(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_CoralHolder p_coralHolder, SUB_ElevatorPivot p_elevatorPivot, 
                     GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_coralHolder = p_coralHolder;
        m_elevatorPivot = p_elevatorPivot;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        switch(m_variables.getRobotState()){
           case HOME:
                //ready, no object
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,Ready()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY))
                ).schedule();
                break;
            case READY:
                //ready to intake
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_INTAKE))
                    ,new CMD_ReadyToIntake(m_elevator, m_wrist, m_elevatorPivot, m_coralHolder)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_INTAKE))
                    ,new CMD_IntakeStow(m_coralHolder, m_wrist, m_elevatorPivot, m_elevator, m_coralHolder)
                    ,Ready()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_STOWED))
                ).schedule();
                break;
            case READY_TO_INTAKE:
                //ready with an object to score
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,Ready()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_STOWED))
                ).schedule();
                break;
           case READY_STOWED:
                //ready to deploy
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                    ,new CMD_ReadyToDeploy(m_elevator, m_coralHolder, m_wrist, m_elevatorPivot)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_DEPLOY))
                ).schedule();
                break;
            case READY_TO_DEPLOY:
                //deploy/score
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                    ,new CMD_Deploy(m_coralHolder, m_wrist, m_elevatorPivot)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.DEPLOY))
                ).schedule();
                break;
            case DEPLOY:
                //ready, no object
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,ReadyHome()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY))
                ).schedule();
                break;
            default:
                break;
        }
    }

    private ConditionalCommand ReadyHome(){
        return new ConditionalCommand(
            new CMD_ReadyHome(m_elevator, m_wrist, m_elevatorPivot, m_coralHolder)
            ,new CMD_ReadyHomeDefensive(m_elevator, m_wrist, m_elevatorPivot, m_coralHolder)
            ,()-> m_variables.isMode(Mode.OFFENSIVE)
        );
    }

    private ConditionalCommand Ready(){
        return new ConditionalCommand(
            new CMD_Ready(m_elevator, m_wrist, m_elevatorPivot, m_coralHolder)
            ,new CMD_ReadyDefensive(m_elevator, m_wrist, m_elevatorPivot, m_coralHolder)
            ,()-> m_variables.isMode(Mode.OFFENSIVE)
        );
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
