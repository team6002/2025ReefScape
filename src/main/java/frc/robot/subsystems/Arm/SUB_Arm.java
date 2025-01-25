package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Arm extends SubsystemBase{
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    public SUB_Arm(ArmIO io){
        this.io = io;
    }

    public void setReference(double p_reference){
      io.setReference(p_reference);
    }

    public double getReference(){
      return inputs.m_armReference;
    }

    public double getPosition(){
      return inputs.m_armPosition;
    }

    public double getCurrent(){
      return inputs.m_armCurrent;
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Arm", inputs);
      io.PID();
    }
}
