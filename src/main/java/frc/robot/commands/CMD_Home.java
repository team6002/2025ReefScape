package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.*;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class CMD_Home extends SequentialCommandGroup{
    public CMD_Home(SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, 
        SUB_Pivot p_pivot, SUB_GroundPivot p_groundPivot, SUB_GroundIntake p_groundIntake,GlobalVariables p_variables){

        addRequirements(p_elevator, p_intake, p_pivot, p_flippyWrist);
        addCommands(
            new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kOff))
            ,new InstantCommand(()-> p_intake.setConveyorVoltage(IntakeConstants.kConveyorOff))
            ,new InstantCommand(()-> p_groundIntake.setVoltage(GroundIntakeConstants.kOff))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kResetElevator))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_groundPivot.setGoal(GroundPivotConstants.kStart))
            ,new WaitCommand(.5)
            ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kHome))
            ,new InstantCommand(()-> p_spinnyWrist.setGoal(FlippyWristConstants.kHome))
            ,new CMD_WristInPosition(p_flippyWrist)
            ,new CMD_ElevatorReset(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kHome))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.HOME))
        );
    }
}
