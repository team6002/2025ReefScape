package frc.robot.subsystems.Winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public static class WinchIOInputs {
    public double m_winchCurrent;
    public double m_winchPos;
  }

  public default void updateInputs(WinchIOInputs inputs) {}

  public default void setReference(double p_reference){}
}
