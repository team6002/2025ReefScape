// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.GroundIntake.SUB_GroundIntake;

// detects when the ground intake sees something
public class CMD_GroundIntakeDetect extends Command {
  /** Creates a new GroundIntakeDetect. */
  SUB_GroundIntake m_groundIntake;
  CommandXboxController m_driverController;
  public CMD_GroundIntakeDetect(SUB_GroundIntake p_groundIntake, CommandXboxController p_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_groundIntake = p_groundIntake;
    m_driverController = p_driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_groundIntake.getCurrent() >= 30){
      m_driverController.setRumble(RumbleType.kBothRumble, .5);
    }else{
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
