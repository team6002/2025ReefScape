package frc.robot.subsystems.CoralHolder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SUB_CoralHolder extends SubsystemBase{
  private final CoralHolderIO io;
  final SysIdRoutine sysIdRoutine;
  private final CoralHolderIOInputsAutoLogged inputs = new CoralHolderIOInputsAutoLogged();
  public SUB_CoralHolder(CoralHolderIO io){
      this.io = io;
      // Create the SysId routine
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

  public void setReference(double p_rpm){
    io.setReference(p_rpm);
  }

  public double getReference(){
    return inputs.m_intakeReference;
  }

  public double getVelocity(){
    return inputs.m_intakeVelocity;
  }

  public double getCurrent(){
    return inputs.m_intakeCurrent;
  }

  public void setVoltage(double p_voltage){
    io.setVoltage(p_voltage);
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
    io.PID();
    Logger.processInputs("CoralHolder", inputs);

    SmartDashboard.putNumber("intake speed", getVelocity());
    SmartDashboard.putNumber("intake goal", getReference());
    SmartDashboard.putNumber("intake current", getCurrent());
  }
  
  public Command sysIdQuasiStatic(SysIdRoutine.Direction direction){
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return sysIdRoutine.dynamic(direction);
  }
}
