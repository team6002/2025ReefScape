package frc.robot.subsystems.Winch;

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
}
