package frc.robot.subsystems.Winch;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Winch extends SubsystemBase{
    private final WinchIO io;
    private final WinchIOInputsAutoLogged inputs = new WinchIOInputsAutoLogged();
    public SUB_Winch(WinchIO io){
        this.io = io;
    }

    public void setPower(double p_power){
        io.setPower(p_power);
    }

    public void setReference(double p_reference){
        io.setReference(p_reference);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("winch", inputs);
    }
}
