package frc.robot.subsystems.GroundPivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundPivotConstants;

public class SUB_GroundPivot extends SubsystemBase{
    private final GroundPivotIO io;
    private final GroundPivotIOInputsAutoLogged inputs = new GroundPivotIOInputsAutoLogged();
    public SUB_GroundPivot(GroundPivotIO io){
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

    public double getVelocity(){
        return io.getVelocity();
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

    public boolean inPosition(double p_position){
      return io.inPosition(p_position);
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("GroundPivot", inputs);
      io.PID();
    }
}
