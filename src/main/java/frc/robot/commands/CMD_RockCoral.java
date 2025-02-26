package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_RockCoral extends SequentialCommandGroup{
    public CMD_RockCoral(SUB_Wrist p_wrist, SUB_Elevator p_elevator, SUB_Pivot p_pivot){
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(()-> p_wrist.setGoal(WristConstants.kHome))
                    ,new CMD_WristInPosition(p_wrist)
                    ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kReady))
                )
                ,new InstantCommand()
                ,()->p_pivot.getPosition() >= PivotConstants.kReadyIntake && p_elevator.getPosition() > ElevatorConstants.kReady
            )
        );
    }
}
