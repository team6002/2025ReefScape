package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Vision.SUB_Vision;

public class AUTO_BlueLeft444 extends SequentialCommandGroup{
    public AUTO_BlueLeft444(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_FlippyWrist p_flippyWrist, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_Algae p_algae, SUB_Vision p_vision){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped(AutoConstants.BlueLeft1), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped(AutoConstants.BlueLeft1), p_drivetrain)
            // ,new InstantCommand(()-> p_drivetrain.setStartingAngle(),p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped(AutoConstants.BlueLeft1)
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_pivot, p_intake)
            )
            ,new CMD_AlignColorAuto(p_drivetrain, p_vision).withTimeout(1)
            ,new WaitCommand(.4)
            ,new CMD_DeployLevelFour(p_intake, p_flippyWrist)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new WaitCommand(.5)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped(AutoConstants.BlueLeft2)
              ,new SequentialCommandGroup(
                new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_pivot, p_intake).withTimeout(5)
                ,new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_pivot, p_intake).withTimeout(5)
              )
            )
            ,new ParallelCommandGroup(
              new CMD_IntakeStow(p_intake).withTimeout(15)
              ,new CMD_DriveForwards(p_drivetrain, 0)
            )
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped("BlueLeftTrio3"), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio3")
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_pivot, p_intake)
            )
            ,new CMD_AlignColorAuto(p_drivetrain, p_vision).withTimeout(1)
            ,new WaitCommand(.4)
            ,new CMD_DeployLevelFour(p_intake, p_flippyWrist)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new WaitCommand(.5)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio4")
              ,new SequentialCommandGroup(
                new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_pivot, p_intake).withTimeout(5)
                ,new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_pivot, p_intake).withTimeout(5)
              )
            )
            ,new ParallelCommandGroup(
              new CMD_IntakeStow(p_intake).withTimeout(1.5)
              ,new CMD_DriveForwards(p_drivetrain, 0)
            )
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped("BlueLeftTrio5"), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio5")
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_pivot, p_intake)
            )
            ,new CMD_AlignColorAuto(p_drivetrain, p_vision).withTimeout(1)
            ,new WaitCommand(.4)
            ,new CMD_DeployLevelFour(p_intake, p_flippyWrist)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio6")
              ,new SequentialCommandGroup(
                new CMD_ReadyIntake(p_elevator, p_flippyWrist, p_pivot, p_intake).withTimeout(5)
                ,new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_pivot, p_intake).withTimeout(5)
              )
            )
            ,new ParallelCommandGroup(
              new CMD_IntakeStow(p_intake).withTimeout(10)
              ,new CMD_DriveForwards(p_drivetrain, 0)
            )
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped("BlueLeftTrio7"), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio7")
              // ,new CMD_ReadyToDeployLevelTwo(p_elevator, p_flippyWrist, p_pivot)
            )
            // ,new CMD_DeployLevelTwo(p_intake, p_flippyWrist)

            // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntake))
            /////// ,new CMD_Ready(p_elevator, p_flippyWrist, p_pivot, p_intake)
        );
    }
}
