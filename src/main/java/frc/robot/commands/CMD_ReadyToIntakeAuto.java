package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_ReadyToIntakeAuto extends SequentialCommandGroup{
    public CMD_ReadyToIntakeAuto(SUB_Elevator p_elevator, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, SUB_Pivot p_pivot, SUB_Intake p_intake){
        addCommands(
            // new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kBelowIntake))
            new ParallelCommandGroup(
                new InstantCommand(()->p_intake.setConveyorVoltage(0))
                ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kIntake))
                ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kIntake))        
                ,new SequentialCommandGroup(        
                new InstantCommand(()-> p_elevator.setConstraints(ElevatorConstants.kSlowVel, ElevatorConstants.kSlowAccel))
                ,new WaitCommand(.3)
                ,new InstantCommand(()-> p_elevator.setConstraints(ElevatorConstants.kMaxVelDown, ElevatorConstants.kMaxAccelDown))
                )
            )
            ,new CMD_WristInPosition(p_flippyWrist)
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_spinnyWrist.setGoal(SpinnyWristConstants.kIntake))
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kIntake))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntake))
            ,new CMD_PivotInPosition(p_pivot)

        );
    }
}