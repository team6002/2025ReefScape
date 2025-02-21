package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SUB_Algae extends SubsystemBase{
    private final AlgaeIO io;
    final SysIdRoutine sysIdRoutine;
    private final AlgaeIoInputsAutoLogged inputs = new AlgaeIoInputsAutoLogged();
    public SUB_Algae(AlgaeIO io){
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

    public void setReference(double p_voltage){
        io.setReference(p_voltage);
    }

    public double getVelocity(){
        return io.getVelocity();
    }

    public double getCurrent(){
        return io.getCurrent();
    }

    public double getReference(){
        return io.getReference();
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
        io.PID();
    }

    public Command sysIdQuasiStatic(SysIdRoutine.Direction direction){
      return sysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return sysIdRoutine.dynamic(direction);
    }
}
