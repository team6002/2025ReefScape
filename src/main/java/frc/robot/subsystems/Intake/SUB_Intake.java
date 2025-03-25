package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public SUB_Intake(IntakeIO io){
        this.io = io;
    }

    public double getReference(){
      return inputs.m_intakeReference;
    }

    public double getVelocity(){
      return inputs.m_intakeVelocity;
    }

    public double getCurrent(){
      return inputs.m_intakeCurrent;
    }

    public void setVoltage(double p_voltage){
      io.setVoltage(p_voltage);
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      io.PID();
      Logger.processInputs("CoralHolder", inputs);

      SmartDashboard.putNumber("intake speed", getVelocity());
      SmartDashboard.putNumber("intake goal", getReference());
      SmartDashboard.putNumber("intake current", getCurrent());
    }
}
