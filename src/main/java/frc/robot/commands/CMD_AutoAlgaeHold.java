package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_AutoAlgaeHold extends SequentialCommandGroup{
    public CMD_AutoAlgaeHold(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist,  SUB_Intake p_intake){
        addCommands(
        new InstantCommand(()->p_pivot.setGoal(PivotConstants.kAlgaeHoldAuto))
        ,new InstantCommand(()->p_intake.setVoltage(IntakeConstants.kAlgaeHolding))
        ,new CMD_PivotInPosition(p_pivot)
   
        );
    }
}
