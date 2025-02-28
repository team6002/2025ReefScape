package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.GlobalVariables.AlgaeTarget;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeLevelThree extends SequentialCommandGroup{
    public CMD_AlgaeLevelThree(SUB_CoralHolder p_intake, SUB_Wrist p_wrist, SUB_Algae p_algae,
        SUB_Elevator p_elevator, GlobalVariables p_variables, SUB_Pivot p_pivot){
        addCommands(
            new CMD_ReadyToIntakeAlgaeThree(p_wrist, p_pivot, p_elevator, p_algae, p_variables)
            ,new CMD_AlgaeTrigger(p_algae, p_variables)
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kHolding))
            ,new InstantCommand(()-> GlobalVariables.m_haveAlgae = true)
            // ,new ConditionalCommand(
            //     new SequentialCommandGroup(
            //         new CMD_DeployLevelTwo(p_intake, p_wrist),
            //         new WaitCommand(.2)
            //         ,new InstantCommand(()-> p_intake.setVoltage(0))
            //     )
            //     ,new PrintCommand("AGABGADA")// ,new CMD_ReadyAlgae(p_elevator, p_wrist, p_pivot, p_variables)
            // ,()-> GlobalVariables.m_haveCoral && GlobalVariables.m_haveAlgae)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgael3))
            ,new CMD_PivotInPosition(p_pivot)
            // ,new ConditionalCommand(
            //     new CMD_YeetAlgae(p_wrist, p_algae)
            //     ,new InstantCommand()
            //     ,()-> GlobalVariables.m_algaeExceptionMode
            // )
            );
    }
}
