package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm.SUB_Arm;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;

public class CMD_ReadyToDeploy extends SequentialCommandGroup{
    public CMD_ReadyToDeploy(SUB_Elevator p_elevator, SUB_CoralHolder p_coralHolder, SUB_Arm p_arm){
        addCommands(
            new InstantCommand(()-> p_elevator.setPivotGoal(ElevatorConstants.kPivotDeploy))
            ,new CMD_PivotInPosition(p_elevator)
            ,new InstantCommand(()-> p_arm.setReference(ArmConstants.kDeploy))
            ,new CMD_ElevatorSetDeploy(p_elevator)
        );
    }
}