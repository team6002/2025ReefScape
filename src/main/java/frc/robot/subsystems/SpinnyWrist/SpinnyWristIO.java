package frc.robot.subsystems.SpinnyWrist;

import org.littletonrobotics.junction.AutoLog;

public interface SpinnyWristIO {
  @AutoLog
  public static class SpinnyWristIOInputs {
    public double m_spinnyWristCurrent;
    public double m_spinnyWristPosition;
    public double m_spinnyWristGoal;
    public boolean m_spinnyWristInPosition;
    public double m_spinnyWristSetpoint;
  }

  public default void updateInputs(SpinnyWristIOInputs inputs) {}

  public default void setGoal(double p_goal){}

  public default void setGoal(){}

  public default void setConstraints(double velocity, double acceleration){};

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default double getSetpoint(){return 0;}

  public default boolean inPosition(){return false;}

  public default boolean inPosition(double p_position){return false;}

  public default void PID(){}

  public default void reset(){

  }
}
