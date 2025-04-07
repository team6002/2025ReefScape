package frc.robot.subsystems.GroundIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntakeIOInputs {
    public double m_groundIntakeCurrent;
    public double m_groundIntakeVelocity;
    public double m_groundIntakeReference;
    public boolean m_leftPressed;
    public boolean m_rightPressed;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GroundIntakeIOInputs inputs) {}

  public default double getCurrent(){return 0;}

  public default double getReference(){return 0;}

  public default double getVelocity(){return 0;}

  public default void setVoltage(double p_voltage){}

  public default boolean hasLeftCoral(){return false;}

  public default boolean hasRightCoral(){return false;}

  public default void PID(){}
}
