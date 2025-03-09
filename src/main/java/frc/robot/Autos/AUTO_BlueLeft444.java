package frc.robot.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class AUTO_BlueLeft444 extends SequentialCommandGroup{
    public AUTO_BlueLeft444(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_Wrist p_wrist, SUB_Elevator p_elevator, SUB_CoralHolder p_intake, SUB_Algae p_algae){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped(AutoConstants.BlueLeft1), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPositionFlipped(AutoConstants.BlueLeft1), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped(AutoConstants.BlueLeft1)
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_wrist, p_pivot, p_intake)
            )
            ,new CMD_DeployLevelFour(p_intake, p_wrist)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new WaitCommand(.1)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped(AutoConstants.BlueLeft2)
              ,new SequentialCommandGroup(
                new CMD_ReadyIntakeAuto(p_elevator, p_wrist, p_pivot, p_intake)
                ,new CMD_ReadyToIntake(p_elevator, p_wrist, p_pivot, p_intake)
              )
            )
            ,new CMD_IntakeStow(p_intake).withTimeout(5)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio3")
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_wrist, p_pivot, p_intake)
            )
            ,new CMD_DeployLevelFour(p_intake, p_wrist)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new WaitCommand(.1)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio4")
              ,new SequentialCommandGroup(
                new CMD_ReadyIntakeAuto(p_elevator, p_wrist, p_pivot, p_intake)
                ,new CMD_ReadyToIntake(p_elevator, p_wrist, p_pivot, p_intake)
              )
            )
            ,new CMD_IntakeStow(p_intake).withTimeout(5)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio5")
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_wrist, p_pivot, p_intake)
            )
            ,new CMD_DeployLevelFour(p_intake, p_wrist)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio6")
              ,new SequentialCommandGroup(
                new CMD_ReadyIntake(p_elevator, p_wrist, p_pivot, p_intake)
                ,new CMD_ReadyToIntake(p_elevator, p_wrist, p_pivot, p_intake)
              )
            )
            ,new CMD_IntakeStow(p_intake).withTimeout(5)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPathFlipped("BlueLeftTrio7")
              ,new CMD_ReadyToDeployLevelTwo(p_elevator, p_wrist, p_pivot)
            )
            ,new CMD_DeployLevelTwo(p_intake, p_wrist)
            ,new InstantCommand(()-> p_drivetrain.setHeading(
                new Rotation2d(Math.toRadians(p_drivetrain.getAngle())).plus(new Rotation2d(Math.PI)).getDegrees()
            ))
            ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntake))
            ///////// ,new CMD_Ready(p_elevator, p_wrist, p_pivot, p_intake)
        );
    }
}
