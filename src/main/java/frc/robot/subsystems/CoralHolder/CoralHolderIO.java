package frc.robot.subsystems.CoralHolder;

import org.littletonrobotics.junction.AutoLog;

public interface CoralHolderIO {
  @AutoLog
  public static class CoralHolderIOInputs {
    public double m_intakeCurrent;
    public double m_intakeVelocity;
    public double m_intakeReference;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CoralHolderIOInputs inputs) {}

  public default double getCurrent(){return 0;}

  public default double getReference(){return 0;}

  public default double getVelocity(){return 0;}

  public default void setVoltage(double p_voltage){}

  public default void PID(){}
}
