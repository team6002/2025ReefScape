package frc.robot.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.GlobalVariables;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.commands.CMD_AlgaeDeploy;
import frc.robot.commands.CMD_AlgaeDeployAuto;
import frc.robot.commands.CMD_AutoAlgaeHold;
import frc.robot.commands.CMD_AutoAlgaeLevel2;
import frc.robot.commands.CMD_AutoAlgaeLevel3;
import frc.robot.commands.CMD_AutoBarge;
import frc.robot.commands.CMD_AutoHome;
import frc.robot.commands.CMD_Deploy;
import frc.robot.commands.CMD_DeployLevelFour;
import frc.robot.commands.CMD_ElevatorInPosition;
import frc.robot.commands.CMD_PivotInPosition;
import frc.robot.commands.CMD_ReadyLevelFourAuto;
import frc.robot.commands.CMD_ReadyToIntakeAuto;
import frc.robot.commands.CMD_WristInPosition;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.SpinnyWrist.SUB_SpinnyWrist;

public class AUTO_MiddleLeft extends SequentialCommandGroup{
    public AUTO_MiddleLeft(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_FlippyWrist p_flippyWrist, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_SpinnyWrist p_spinnyWrist, GlobalVariables p_variables){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("Middle1"), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("Middle1"), p_drivetrain)
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kHolding))
            ,new ParallelCommandGroup(
                p_drivetrain.FollowPath("Middle1")
                ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            )
            ,new ParallelCommandGroup(
                new CMD_PivotInPosition(p_pivot)
                ,new CMD_WristInPosition(p_flippyWrist)
            )
            ,new CMD_Deploy(p_intake, p_flippyWrist, p_variables)// , new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            ,new SequentialCommandGroup(    
                    new ParallelCommandGroup(
                        new InstantCommand(()->p_spinnyWrist.setGoal(SpinnyWristConstants.kIntake))
                        ,new InstantCommand(()->p_flippyWrist.setGoal(FlippyWristConstants.kReadyIntakeAlgae))
                        ,new InstantCommand(()->p_pivot.setGoal(PivotConstants.kReadyAlgaeLvl2Auto))
                        // ,new SequentialCommandGroup(
                        //     new WaitCommand(.05) 
                        ,new InstantCommand(()->p_elevator.setGoal(ElevatorConstants.kReadyIntakeAlgael2Down))
                        ,new InstantCommand(()->p_intake.setVoltage(IntakeConstants.kReverse))
                    )
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new CMD_WristInPosition(p_flippyWrist) 
            )
            ,p_drivetrain.FollowPath("Middle2")
            ,new CMD_AutoAlgaeLevel2(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)
            ,new ParallelCommandGroup(
                p_drivetrain.FollowPath("Middle3")
                ,new SequentialCommandGroup(
                    new CMD_AutoAlgaeHold(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)        
                    ,new CMD_AutoBarge(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)
                )
            )
            ,new CMD_AlgaeDeployAuto(p_intake, p_flippyWrist, p_variables)
            , new ParallelCommandGroup(
                p_drivetrain.FollowPath("Middle4")
                ,new CMD_AutoAlgaeLevel3(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)        
            )
            ,new ParallelCommandGroup(
                p_drivetrain.FollowPath("Middle5")
                ,new SequentialCommandGroup(
                    new CMD_AutoAlgaeHold(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)
                    ,new CMD_AutoBarge(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)
                )
            )
            ,new CMD_AlgaeDeployAuto(p_intake, p_flippyWrist, p_variables)
            ,new CMD_AutoHome(p_pivot, p_elevator, p_flippyWrist, p_spinnyWrist, p_intake)
            ,p_drivetrain.FollowPath("Middle6")
        );
    }
}
