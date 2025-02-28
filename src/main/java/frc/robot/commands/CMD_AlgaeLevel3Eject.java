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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_AlgaeLevel3Eject extends SequentialCommandGroup{
    public CMD_AlgaeLevel3Eject(SUB_CoralHolder p_intake, SUB_Wrist p_wrist, SUB_Algae p_algae,
        SUB_Elevator p_elevator, GlobalVariables p_variables, SUB_Pivot p_pivot){
        addCommands(
            new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kIntake))
            // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyAlgae))
            // ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReadyAlgael3))
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgaeEject))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReadyIntakeAlgael3))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> GlobalVariables.lvl3AlgaeException = false)
            ,new InstantCommand(()-> p_algae.setReference(0)) 
            );
    }
}
