package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

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
      return io.getGoal();
    }

    public double getPosition(){
      return io.getPosition();
    }

    public double getCurrent(){
      return io.getCurrent();
    }

    public void reset(){
      io.reset();
    }

    public void setConstraints(double velocity, double acceleration){
      io.setConstraints(velocity, acceleration);
    }
    public double getSetpoint(){
      return io.getSetpoint();
    }

    public boolean inPosition(){
      return io.inPosition();
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Wrist", inputs);
      io.PID();
    }
}
