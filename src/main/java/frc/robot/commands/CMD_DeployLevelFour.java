package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Intake.SUB_Intake;
import frc.robot.subsystems.Wrist.SUB_Wrist;

public class CMD_DeployLevelFour extends SequentialCommandGroup{
    public CMD_DeployLevelFour(SUB_Intake p_intake, SUB_Wrist p_wrist){
        addCommands(
            new InstantCommand(()-> p_wrist.setGoal(WristConstants.kStowing))
            ,new InstantCommand(()-> p_intake.setVoltage(IntakeConstants.kReverseLvl4))
            ,new CMD_WristInPosition(p_wrist)
        );
    }
}
