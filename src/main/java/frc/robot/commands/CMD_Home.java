package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.Arm.SUB_Arm;
import frc.robot.subsystems.CoralHolder.SUB_CoralHolder;
import frc.robot.subsystems.Elevator.SUB_Elevator;
import frc.robot.subsystems.ElevatorPivot.SUB_ElevatorPivot;

public class CMD_Home extends SequentialCommandGroup{
    public CMD_Home(SUB_Elevator p_elevator, SUB_CoralHolder p_coralHolder, SUB_Arm p_arm, SUB_ElevatorPivot p_elevatorPivot){
        addCommands(
            new InstantCommand(()-> p_coralHolder.setReference(CoralHolderConstants.kOff))
            ,new InstantCommand(()->p_arm.setGoal(ArmConstants.kHome))
            ,new CMD_ArmInPosition(p_arm)
            ,new InstantCommand(()-> p_elevator.setGoal(ElevatorConstants.kHome))
            ,new CMD_ElevatorInPosition(p_elevator)
            ,new InstantCommand(()-> p_elevatorPivot.setGoal(ElevatorPivotConstants.kHome))
            ,new CMD_PivotInPosition(p_elevatorPivot)
        );
    }
}
