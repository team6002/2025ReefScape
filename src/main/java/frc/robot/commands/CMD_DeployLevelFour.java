package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.subsystems.FlippyWrist.SUB_FlippyWrist;
import frc.robot.subsystems.Intake.SUB_Intake;

public class CMD_DeployLevelFour extends SequentialCommandGroup{
    public CMD_DeployLevelFour(SUB_Intake p_intake, SUB_FlippyWrist p_flippyWrist){
        addCommands(
            new InstantCommand(()-> p_flippyWrist.setGoal(FlippyWristConstants.kStowing))
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kReverseLvl4))
            ,new CMD_WristInPosition(p_flippyWrist)
        );
    }
}
