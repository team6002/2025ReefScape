package frc.robot.subsystems.FlippyWrist;

import org.littletonrobotics.junction.AutoLog;

public interface FlippyWristIO {
  @AutoLog
  public static class FlippyWristIOInputs {
    public double m_flippyWristCurrent;
    public double m_flippyWristPosition;
    public double m_flippyWristGoal;
    public boolean m_flippyWristInPosition;
    public double m_flippyWristSetpoint;
  }

  public default void updateInputs(FlippyWristIOInputs inputs) {}

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
