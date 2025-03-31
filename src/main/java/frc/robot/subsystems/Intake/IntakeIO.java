package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double m_intakeCurrent;
    public double m_intakeVelocity;
    public double m_intakeReference;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default double getCurrent(){return 0;}

  public default double getReference(){return 0;}

  public default double getVelocity(){return 0;}

  public default void setVoltage(double p_voltage){}

  public default void setReference(double p_speed){}

  public default void setConveyorVoltage(double p_voltage){}
  
  public default void setCurrentLimit(int p_limit){}
    
}
