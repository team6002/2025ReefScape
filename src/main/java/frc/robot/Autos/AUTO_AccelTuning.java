package frc.robot.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Pivot.SUB_Pivot;

public class AUTO_AccelTuning extends SequentialCommandGroup{
    public AUTO_AccelTuning(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_FlippyWrist p_flippyWrist, SUB_Elevator p_elevator, SUB_Intake p_intake){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("AccelTune1"), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("AccelTune1"), p_drivetrain)
            ,p_drivetrain.FollowPath("AccelTune1")
            , new WaitCommand(2)
            ,p_drivetrain.FollowPath("AccelTune2")
            ,new InstantCommand(()-> p_drivetrain.setHeading(
                new Rotation2d(Math.toRadians(p_drivetrain.getAngle())).plus(new Rotation2d(Math.PI)).getDegrees()
            ))
            
        );
    }
}
