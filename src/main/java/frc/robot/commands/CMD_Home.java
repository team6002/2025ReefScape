package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.GlobalVariables.RobotState;
import frc.robot.Constants.*;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class CMD_Home extends SequentialCommandGroup{
    public CMD_Home(SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist, SUB_Pivot p_pivot, SUB_Algae p_algae, GlobalVariables p_variables){
        addRequirements(p_algae, p_elevator, p_intake, p_pivot, p_flippyWrist);
        addCommands(
            new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kOff))
            ,new InstantCommand(()-> p_algae.setReference(AlgaeConstants.kOff))
            ,new WaitCommand(.5)
            ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kHome))
            ,new CMD_WristInPosition(p_flippyWrist)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kReady))
            ,new CMD_PivotInPosition(p_pivot)
            ,new CMD_ElevatorReset(p_elevator)
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kHome))
            ,new CMD_PivotInPosition(p_pivot)
            ,new InstantCommand(()-> p_variables.setRobotState(RobotState.HOME))
        );
    }
}
