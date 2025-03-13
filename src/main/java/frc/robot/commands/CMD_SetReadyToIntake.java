package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyToIntake extends Command{
    private final SUB_Elevator m_elevator;
    private final SUB_Wrist m_wrist;
    private final SUB_Pivot m_pivot;
    private final SUB_CoralHolder m_intake;
    private final GlobalVariables m_variables;
    public CMD_SetReadyToIntake(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, 
        GlobalVariables p_variables){
        
        m_elevator = p_elevator;
        m_wrist = p_wrist;
        m_pivot = p_pivot;
        m_intake = p_intake;
        m_variables = p_variables;
    }

    @Override
    public void initialize(){
        new SequentialCommandGroup(
            new InstantCommand(()-> m_variables.setRobotState(RobotState.READY_TO_INTAKE))
            ,new ConditionalCommand(
                new CMD_ReadyIntake(m_elevator, m_wrist, m_pivot, m_intake),
                new InstantCommand(),
                ()-> GlobalVariables.m_targetCoralLevel == 4 &! GlobalVariables.m_intakingAlgae &! m_variables.isRobotState(RobotState.HOME)
            )
            ,getIntakeCommand()
            ,new InstantCommand(()-> GlobalVariables.m_intakingAlgae = false)
            ,new CMD_IntakeStow(m_intake)
            ,new InstantCommand(()-> GlobalVariables.m_haveCoral = true)
        ).schedule();
    }

    private SequentialCommandGroup getIntakeCommand(){
        SequentialCommandGroup intakeCommand = new CMD_ReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake);

        if(GlobalVariables.m_intakingAlgae){
            switch (m_variables.getAlgaeTarget()) {
                case BARGE:
                    intakeCommand = new CMD_ReadyToIntakeFromBarge(m_elevator, m_wrist, m_pivot, m_intake);
                    break;
                case PROCESSOR:
                    intakeCommand = new CMD_ReadyToIntakeFromProcessor(m_elevator, m_wrist, m_pivot, m_intake);
                    break;
                default:
                    break;
            }
        }else{
            intakeCommand = new CMD_ReadyToIntake(m_elevator, m_wrist, m_pivot, m_intake);
        }

        return intakeCommand; 
    }

    @Override
    public boolean isFinished(){
        return GlobalVariables.m_haveCoral;
    }
}
