package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_ReadyToIntakeAuto extends SequentialCommandGroup{
    public CMD_ReadyToIntakeAuto(SUB_Elevator p_elevator, SUB_Wrist p_wrist, SUB_Pivot p_pivot, SUB_Intake p_intake){
        addCommands(
            // new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kBelowIntake))
            new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kIntake))
            ,new InstantCommand(()-> p_wrist.setGoal(WristConstants.kIntake))
            ,new WaitCommand(.2)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntake))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kIntake))
            ,new CMD_WristInPosition(p_wrist)
            ,new CMD_PivotInPosition(p_pivot)

        );
    }
}