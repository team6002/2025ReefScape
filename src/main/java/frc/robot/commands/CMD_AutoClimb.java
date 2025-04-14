package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GroundPivotConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.subsystems.Drive.SUB_Drivetrain;
import frc.robot.subsystems.GroundPivot.SUB_GroundPivot;
import frc.robot.subsystems.Winch.SUB_Winch;

public class CMD_AutoClimb extends Command{
    private final SUB_Drivetrain m_drivetrain;
    private final SUB_Winch m_winch;
    private final CommandXboxController m_driverController;
    private final SUB_GroundPivot m_groundPivot;
    private boolean isFinshed;
    public CMD_AutoClimb(SUB_Drivetrain p_drivetrain, SUB_Winch p_winch, CommandXboxController p_driverController, SUB_GroundPivot p_groundPivot){
        m_drivetrain = p_drivetrain;
        m_winch = p_winch;
        m_driverController = p_driverController;
        m_groundPivot = p_groundPivot;
        addRequirements(m_drivetrain);
        isFinshed = false;
    }

    @Override
    public void initialize(){
        isFinshed = false;
        m_groundPivot.setCurrentLimit(10);
    }

    @Override
    public void execute(){
        m_drivetrain.drive(.15, 0, 0, false);
        if(Math.abs(m_drivetrain.getPitch()) > 3 || m_driverController.a().getAsBoolean())
        {
            m_drivetrain.drive(0, 0, 0, false);
            m_winch.setReference(WinchConstants.kClimb);
            isFinshed = true;
        }
    }


    @Override
    public void end(boolean interrupted){
        m_groundPivot.setGoal(GroundPivotConstants.kClimbUp);
    }

    @Override
    public boolean isFinished(){
        return isFinshed
         || Math.abs(m_driverController.getLeftX()) > .1 || Math.abs(m_driverController.getLeftY()) > .1 || Math.abs(m_driverController.getRightX()) > .1;
    }
}
