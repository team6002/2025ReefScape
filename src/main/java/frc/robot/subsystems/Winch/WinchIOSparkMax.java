package frc.robot.subsystems.Winch;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;

public class WinchIOSparkMax implements WinchIO{
    private final SparkMax m_winchMotor;
    public WinchIOSparkMax(){
        m_winchMotor = new SparkMax(HardwareConstants.kWinchCanId, MotorType.kBrushless);

        m_winchMotor.configure(Configs.WinchConfig.m_WinchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(WinchIOInputs inputs){
        inputs.m_WinchCurrent = m_winchMotor.getOutputCurrent();
    }

    @Override
    public void setPower(double p_power){
        m_winchMotor.set(p_power);
    }
}
