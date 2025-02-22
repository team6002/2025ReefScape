package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class AUTO_BlueRight extends SequentialCommandGroup{
    public AUTO_BlueRight(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_Wrist p_wrist, SUB_Elevator p_elevator, SUB_CoralHolder p_intake){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition(AutoConstants.BlueLeft1), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition(AutoConstants.BlueLeft1), p_drivetrain)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath(AutoConstants.BlueLeft1)
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_wrist, p_pivot, p_intake)
            )
            ,new CMD_DeployLevelFour(p_intake, p_wrist)
            ,new WaitCommand(.2)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath(AutoConstants.BlueLeft2)
              ,new SequentialCommandGroup(
                new CMD_Ready(p_elevator, p_wrist, p_pivot, p_intake)
                ,new CMD_ReadyToIntake(p_elevator, p_wrist, p_pivot, p_intake)
              )
            )
            ,new CMD_IntakeStow(p_intake).withTimeout(1)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath(AutoConstants.BlueLeft3)
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_wrist, p_pivot, p_intake)
            )
            ,new CMD_DeployLevelFour(p_intake, p_wrist)
            ,new WaitCommand(.2)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath(AutoConstants.BlueLeft4)
              ,new SequentialCommandGroup(
                new CMD_Ready(p_elevator, p_wrist, p_pivot, p_intake)
                ,new CMD_ReadyToIntake(p_elevator, p_wrist, p_pivot, p_intake)
              )
            )
            ,new CMD_IntakeStow(p_intake).withTimeout(1)
            ,new ParallelCommandGroup(
              p_drivetrain.FollowPath(AutoConstants.BlueLeft5)
              ,new CMD_ReadyLevelFourAuto(p_elevator, p_wrist, p_pivot, p_intake)
            )
            ,new CMD_DeployLevelFour(p_intake, p_wrist)
            ,new WaitCommand(.2)
        );
    }
}
