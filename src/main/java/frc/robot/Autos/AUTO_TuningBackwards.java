package frc.robot.Autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Algae.SUB_Algae;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.Pivot.SUB_Pivot;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class AUTO_TuningBackwards extends SequentialCommandGroup{
    public AUTO_TuningBackwards(SUB_Drivetrain p_drivetrain, SUB_Pivot p_pivot, SUB_Wrist p_wrist, SUB_Elevator p_elevator, SUB_CoralHolder p_intake, SUB_Algae p_algae){
        addCommands(
            Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("Tune2"), p_drivetrain)
            ,Commands.runOnce(()-> p_drivetrain.resetOdoToStartPosition("Tune2"), p_drivetrain)
            ,p_drivetrain.FollowPath("Tune2")
            // ,p_drivetrain.FollowPath("Box2")
            ,new InstantCommand(()-> p_drivetrain.setHeading(
                new Rotation2d(Math.toRadians(p_drivetrain.getAngle())).plus(new Rotation2d(Math.PI)).getDegrees()
            ))
            
        );
    }
}
