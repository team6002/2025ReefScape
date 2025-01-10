package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIHolderIO {
  @AutoLog
  public static class IntakeIOInputs {
    
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}
}
