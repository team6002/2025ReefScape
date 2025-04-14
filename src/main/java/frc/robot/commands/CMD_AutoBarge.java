package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_AutoBarge extends SequentialCommandGroup{
    public CMD_AutoBarge(SUB_Pivot p_pivot, SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist,  SUB_Intake p_intake){
        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(()->p_pivot.setGoal(PivotConstants.kDeployBarge))
                ,new InstantCommand(()->p_spinnyWrist.setGoal(SpinnyWristConstants.kIntake))           
                ,new InstantCommand(()->p_flippyWrist.setGoal(FlippyWristConstants.kDeployBargeAuto)) 
            )
            ,new InstantCommand(()->p_elevator.setGoal(ElevatorConstants.kDeployBarge))
            ,new CMD_ElevatorInPosition(p_elevator)
        );
    }
}
