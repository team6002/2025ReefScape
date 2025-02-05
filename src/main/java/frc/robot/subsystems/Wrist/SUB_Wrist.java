package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Wrist extends SubsystemBase{
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    public SUB_Wrist(WristIO io){
        this.io = io;
    }

    public void setGoal(double p_Goal){
      io.setGoal(p_Goal);
    }

    public double getGoal(){
      return inputs.m_wristGoal;
    }

    public double getPosition(){
      return inputs.m_wristPosition;
    }

    public double getCurrent(){
      return inputs.m_wristCurrent;
    }

    public void reset(){
      io.reset();
    }

    public double getSetpoint(){
      return io.getSetpoint();
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Wrist", inputs);
      io.PID();

      SmartDashboard.putNumber("wristVoltage", getCurrent());
      SmartDashboard.putNumber("wristPos", Math.toDegrees(getPosition()));
      SmartDashboard.putNumber("wristGoal", Math.toDegrees(getGoal()));
      SmartDashboard.putNumber("wrist setpoint", Math.toDegrees(getSetpoint()));
    }
}
