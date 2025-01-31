package frc.robot.subsystems.CoralHolder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_CoralHolder extends SubsystemBase{
    private final CoralIHolderIO io;
    private final CoralHolderIOInputsAutoLogged inputs = new CoralHolderIOInputsAutoLogged();
    public SUB_CoralHolder(CoralIHolderIO io){
        this.io = io;
    }

    public void setReference(double p_rpm){
      io.setReference(p_rpm);
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
