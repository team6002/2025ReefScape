package frc.robot.subsystems.CoralHolder;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIHolderIO {
  @AutoLog
  public static class CoralHolderIOInputs {
    public double m_intakeCurrent;
    public double m_intakeVelocity;
    public double m_intakeReference;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CoralHolderIOInputs inputs) {}

  public default void setReference(double p_rpm){}
}
