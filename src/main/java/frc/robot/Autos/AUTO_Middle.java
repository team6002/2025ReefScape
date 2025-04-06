package frc.robot.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.CMD_DeployLevelFour;
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

public class AUTO_Middle extends SequentialCommandGroup{
    public AUTO_Middle(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_FlippyWrist p_flippyWrist, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_SpinnyWrist p_spinnyWrist){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("Middle"), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("Middle"), p_drivetrain)
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kHolding))
            ,new ParallelCommandGroup(
                p_drivetrain.FollowPath("Middle")
                ,new CMD_ReadyLevelFourAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            )
            ,new ParallelCommandGroup(
                new CMD_PivotInPosition(p_pivot)
                ,new CMD_WristInPosition(p_flippyWrist)
            )
            ,new ParallelCommandGroup(
              new CMD_DeployLevelFour(p_intake, p_flippyWrist)
              ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kAutoDown))  
              // ,new InstantCommand(()-> p_elevator.setConstraints(ElevatorConstants.kSlowVel, ElevatorConstants.kSlowAccel))
              // ,new InstantCommand(()-> p_pivot.setGoal(PivotConstants.kAutoDeploy))
              // ,new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kAutoDeploy))
            )
            ,new WaitCommand(.2)
            ,new InstantCommand(()-> p_intake.setConveyorVoltage(0))
            , new CMD_ReadyToIntakeAuto(p_elevator, p_flippyWrist, p_spinnyWrist, p_pivot, p_intake)
            
        );
    }
}
