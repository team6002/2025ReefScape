package frc.robot.subsystems.Winch;

import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public static class WinchIOInputs {
    public double m_WinchCurrent;
  }

  public default void updateInputs(WinchIOInputs inputs) {}

  public default void setPower(double p_power){}
}
