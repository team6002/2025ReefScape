package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.GlobalVariables;

public class SUB_Pivot extends SubsystemBase{
    private final PivotIO io;
    final SysIdRoutine sysIdRoutine;
    private final PivotIoInputsAutoLogged inputs = new PivotIoInputsAutoLogged();
    public SUB_Pivot(PivotIO io){
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
        SmartDashboard.putNumber("pivotGoal", Math.toRadians(getPosition()));
        SmartDashboard.putNumber("pivotPos", Math.toRadians(getPosition()));
        SmartDashboard.putNumber("pivotCurrent", getCurrent());
    }

    public double getPosition(){
      return inputs.m_pivotPos;
    }

    public double getCurrent(){
      return inputs.m_pivotCurrent;
    }

    public double getGoal(){
     return inputs.m_pivotGoal;
    }

    public void setGoal(double p_goal){
      io.setGoal(p_goal);
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

    public Command incrementGoal(double increment){
      return Commands.runOnce(() -> io.setGoal(increment + getGoal()));
    }

    public void setVoltage(double voltage){
      io.setVoltage(voltage);
    }
    
    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Pivot", inputs);
      io.PID();
      // SmartDashboard.putNumber("elevatorPivotPos", Math.toDegrees(getPosition()));
      // SmartDashboard.putNumber("elevatorPivotCurrent", getCurrent());
      // SmartDashboard.putNumber("elevatorPivotGoal", Math.toDegrees(getGoal()));
      // SmartDashboard.putNumber("pivot setpoint", Math.toDegrees(getSetpoint()));
      GlobalVariables.m_pivotAngle = getPosition();
    }
    
    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction){
      return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
      return sysIdRoutine.dynamic(direction);
    }
}