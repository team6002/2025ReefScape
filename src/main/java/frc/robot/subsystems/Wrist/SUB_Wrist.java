package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SUB_Wrist extends SubsystemBase{
    private final WristIO io;
    final SysIdRoutine sysIdRoutine;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    public SUB_Wrist(WristIO io){
        this.io = io;
        sysIdRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(
          null, null, null, // Use default config
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())
          ),
          
          new SysIdRoutine.Mechanism(
          voltage -> {
              setVoltage(voltage.magnitude());
          },
          null, // No log consumer, since data is recorded by AdvantageKit
          this
          )
        );
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

    public boolean inPosition(){
      return io.inPosition();
    }

    public void setVoltage(double voltage){
      io.setVoltage(voltage);
    }
    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Wrist", inputs);
      io.PID();
    }
 
    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction){
      return sysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
      return sysIdRoutine.dynamic(direction);
    }
}
