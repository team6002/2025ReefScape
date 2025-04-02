package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.robot.Configs.IntakeConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;
import frc.robot.subsystems.Vision.SUB_Vision;


//elevator showed sensor fault
//spinny wrist sensor fault
//
public class AUTO_BlueRight444 extends SequentialCommandGroup{
    public AUTO_BlueRight444(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_FlippyWrist p_flippyWrist, SUB_SpinnyWrist p_spinnyWrist, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_Vision p_vision, GlobalVariables p_variables){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition(AutoConstants.BlueLeft1), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition(AutoConstants.BlueLeft1), p_drivetrain)
            // ,new InstantCommand(()-> p_drivetrain.setStartingAngle(),p_drivetrain)
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kHolding))
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath(AutoConstants.BlueLeft1)
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            )
            ,new WaitCommand(.5)
            ,new CMD_PivotInPosition(p_pivot)
            ,new ParallelCommandGroup(
              new CMD_DeployLevelFour(p_intake, p_flippyWrist)
              // ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kAutoDown))  
              // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kAutoDeploy))
              // ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kAutoDeploy))
            )
            ,new WaitCommand(.4)
            ,new InstantCommand(()-> p_intake.setConveyorVoltage(0))
            // ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kReady))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new ParallelCommandGroup(
              new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
              ,p_drivetrain.FollowPath(AutoConstants.BlueLeft2)
            )
            // ,new ParallelCommandGroup(
            ,new CMD_IntakeStow(p_intake).withTimeout(15)
              // ,new CMD_DriveForwards(p_drivetrain, 0)
            // )
            // ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("BlueLeftTrio3"), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath("BlueLeftTrio3")
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            )
            ,new CMD_PivotInPosition(p_pivot)
            // ,new CMD_DeployLevelFour(p_intake, p_flippyWrist)
            // ,new WaitCommand(.3)
            ,new WaitCommand(.5)
            ,new ParallelCommandGroup(
              new CMD_DeployLevelFour(p_intake, p_flippyWrist)
              // ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kAutoDown))
              // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kAutoDeploy))
              // ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kAutoDeploy))
            )
            ,new WaitCommand(.4)
            ,new InstantCommand(()-> p_intake.setConveyorVoltage(0))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new ParallelCommandGroup(
              new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
              ,p_drivetrain.FollowPath("BlueLeftTrio4")
            )
            // ,new ParallelCommandGroup(
            ,new CMD_IntakeStow(p_intake).withTimeout(15)
            //   ,new CMD_DriveForwards(p_drivetrain, 0)
            // )
            // ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("BlueLeftTrio5"), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath("BlueLeftTrio5")
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            )
            // ,new WaitCommand(.4)
            ,new CMD_PivotInPosition(p_pivot)
            ,new CMD_DeployLevelFour(p_intake, p_flippyWrist)
            ,new ParallelCommandGroup(
              new CMD_DeployLevelFour(p_intake, p_flippyWrist)
              // ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kAutoDown))  
              // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kAutoDeploy))
              // ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kAutoDeploy))
            )
            ,new WaitCommand(.4)
            ,new InstantCommand(()-> p_intake.setConveyorVoltage(0))
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath("BlueLeftTrio6")
              ,new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            )
            // ,new ParallelCommandGroup(
            //   new CMD_IntakeStow(p_intake).withTimeout(10)
            //   ,new CMD_DriveForwards(p_drivetrain, 0)
            // )
            // ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("BlueLeftTrio7"), p_drivetrain)
            // ,new ParallelCommandGroup(
            //   p_drivetrain.FollowPath("BlueLeftTrio7")
            //   // ,new CMD_ReadyToDeployLevelTwo(p_elevator, p_flippyWrist, p_pivot)
            // )
            // // ,new CMD_DeployLevelTwo(p_intake, p_flippyWrist)

            // // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kIntake))
            // /////// ,new CMD_Ready(p_elevator, p_flippyWrist, p_pivot, p_intake)
        );
    }
}
