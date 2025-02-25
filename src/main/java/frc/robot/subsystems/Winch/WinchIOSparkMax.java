package frc.robot.subsystems.Winch;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HardwareConstants;

public class WinchIOSparkMax implements WinchIO{
    private final SparkMax m_winchMotor;
    private final SparkClosedLoopController m_winchController;
    private final RelativeEncoder m_winchEncoder;
    public WinchIOSparkMax(){
        m_winchMotor = new SparkMax(HardwareConstants.kWinchCanId, MotorType.kBrushless);

        m_winchController = m_winchMotor.getClosedLoopController();

        m_winchEncoder = m_winchMotor.getEncoder();

        m_winchMotor.configure(Configs.WinchConfig.m_WinchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(WinchIOInputs inputs){
        inputs.m_winchCurrent = m_winchMotor.getOutputCurrent();
        inputs.m_winchPos = m_winchEncoder.getPosition();
    }

    @Override
    public void setReference(double p_reference){
        m_winchController.setReference(p_reference, ControlType.kPosition);
    }
}
