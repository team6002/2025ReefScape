package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.Mode;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.Wrist.SUB_Wrist;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
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
                    ,new CMD_ReadyToDeploy(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_DEPLOY))
                ).schedule();
                break;
            case READY_TO_DEPLOY:
                //deploy/score
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_DEPLOY))
                    ,new CMD_Deploy(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
                    ,new CMD_ElevatorInPosition(m_elevator)
                    ,new CMD_PivotInPosition(m_pivot)
                    ,new CMD_WristInPosition(m_wrist)
                    ,new InstantCommand(()-> GlobalVariables.m_haveCoral = false)
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.DEPLOY))
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    // ,new InstantCommand(()-> m_elevator.setGoal(ElevatorConstants.kIntake))                    
                    ,readyHome()
                    ,new InstantCommand(()-> m_variables.setRobotState(RobotState.READY))
                ).schedule();
                break;
            case DEPLOY:
                //ready, no object
                new SequentialCommandGroup(
                    new InstantCommand(()-> m_variables.setRobotState(RobotState.TRANSITIONING_TO_READY))
                    ,new CMD_Deploy(m_elevator, m_wrist, m_pivot, m_intake, m_variables)
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

    @Override
    public boolean isFinished(){
        return true;
    }
}
