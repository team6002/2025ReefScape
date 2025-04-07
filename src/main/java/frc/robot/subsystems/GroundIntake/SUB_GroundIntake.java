package frc.robot.subsystems.GroundIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_GroundIntake extends SubsystemBase{
    private final GroundIntakeIO io;
    private final GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();
    public SUB_GroundIntake(GroundIntakeIO io){
        this.io = io;
    }

    public double getReference(){
      return inputs.m_groundIntakeReference;
    }

    public double getVelocity(){
      return inputs.m_groundIntakeVelocity;
    }

    public double getCurrent(){
      return inputs.m_groundIntakeCurrent;
    }

    public void setVoltage(double p_voltage){
      io.setVoltage(p_voltage);
    }

    public boolean hasCorral(){
      return io.hasLeftCoral() && io.hasRightCoral();
    }
    
    @Override
    public void periodic(){
      io.updateInputs(inputs);
      io.PID();
      Logger.processInputs("GroundIntake", inputs);

      SmartDashboard.putNumber("intake speed", getVelocity());
      SmartDashboard.putNumber("intake goal", getReference());
      SmartDashboard.putNumber("intake current", getCurrent());
    }
}
