package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Algae extends SubsystemBase{
    private final AlgaeIO io;
    private final AlgaeIoInputsAutoLogged inputs = new AlgaeIoInputsAutoLogged();
    public SUB_Algae(AlgaeIO io){
        this.io = io;
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

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
        io.PID();
    }
}
