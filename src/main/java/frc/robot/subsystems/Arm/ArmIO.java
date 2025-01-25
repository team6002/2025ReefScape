package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double m_armCurrent;
    public double m_armPosition;
    public double m_armGoal;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setGoal(double p_goal){}

  public default void setPid(double kP, double kI, double kD, double kFF){}

  public default void setFeedforward(double kS, double kG, double kV){}

  public default void setGoal(){}

  public default double getGoal(){return 0;}

  public default double getPosition(){return 0;}

  public default double getCurrent(){return 0;}

  public default void PID(){}
}
