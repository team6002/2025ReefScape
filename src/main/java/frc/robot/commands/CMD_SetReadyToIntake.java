package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_SetReadyToIntake extends SequentialCommandGroup{
    
    public CMD_SetReadyToIntake(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_CoralHolder p_intake, 
        GlobalVariables p_variables){
        addCommands(
            new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_TO_INTAKE))
            ,new ConditionalCommand(
                new CMD_ReadyIntake(p_elevator, p_wrist, p_pivot, p_intake),
                new InstantCommand(),
                ()-> GlobalVariables.m_targetCoralLevel == 4 
            )
            ,new CMD_ReadyToIntake(p_elevator, p_wrist, p_pivot, p_intake)
            ,new CMD_IntakeStow(p_intake)
            ,new InstantCommand(()-> GlobalVariables.m_coralException = false)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.READY_STOWED))
            ,new InstantCommand(()-> GlobalVariables.m_haveCoral = true)
            ,new InstantCommand(()-> p_intake.setVoltage(CoralHolderConstants.kHolding))
            ,new ConditionalCommand(
                new InstantCommand()
                ,new InstantCommand(()-> this.cancel())
                ,()-> GlobalVariables.m_haveCoral
            )
            ,new CMD_Ready(p_elevator, p_wrist, p_pivot, p_intake)
        );
    }
}
