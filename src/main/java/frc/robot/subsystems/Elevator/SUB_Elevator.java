package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private final ElevatorIoInputsAutoLogged inputs = new ElevatorIoInputsAutoLogged();
    public SUB_Elevator(ElevatorIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
    }
}
