package frc.robot.subsystems.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.GlobalVariables;
import frc.robot.Configs;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.HardwareConstants;

public class WristIOSparkMax implements WristIO{
    private final SparkMax m_wristMotor;
    private final AbsoluteEncoder m_wristEncoder;
    private final SparkClosedLoopController m_wristController;
    private  ArmFeedforward m_wristFeedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
    private final Constraints m_wristConstraints = new Constraints(WristConstants.kMaxVel, WristConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public WristIOSparkMax(){
        //initialize motor
        m_wristMotor = new SparkMax(HardwareConstants.kWristCanId, MotorType.kBrushless);

        //initialize PID controller
        m_wristController = m_wristMotor.getClosedLoopController();

        //initalize encoder
        m_wristEncoder = m_wristMotor.getAbsoluteEncoder();

        //apply config
        m_wristMotor.configure(Configs.WristConfigs.m_wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset reference in init
        m_setpoint = new TrapezoidProfile.State(getPosition() - WristConstants.kWristOffset, 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(WristIOInputs inputs){
        inputs.m_wristGoal = getGoal();
        inputs.m_wristCurrent = getCurrent();
        inputs.m_wristPosition = getPosition();
        inputs.m_inPosition = inPosition();
    }

    @Override
    public void setGoal(double p_Goal){
        m_setpoint = new TrapezoidProfile.State(getPosition() - WristConstants.kWristOffset, 0);
        m_goal = new TrapezoidProfile.State(p_Goal - WristConstants.kWristOffset, 0);
    }

    @Override
    public double getPosition(){
        return m_wristEncoder.getPosition()+WristConstants.kWristOffset;
    }

    @Override
    public double getCurrent(){
        return m_wristMotor.getOutputCurrent();
    }

    @Override
    public double getGoal(){
        return m_goal.position + WristConstants.kWristOffset;
    }

    @Override public double getSetpoint(){
        return m_setpoint.position + WristConstants.kWristOffset;
    }

    @Override
    public boolean inPosition(){
        return Math.abs(getPosition() - getGoal()) < WristConstants.kTolerance;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_wristConstraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_wristController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_wristFeedforward.calculate(getPosition() - WristConstants.kWristOffset- GlobalVariables.m_pivotAngle, m_setpoint.velocity));
    }

    @Override
    public void reset(){
        m_setpoint = new TrapezoidProfile.State(getPosition() - WristConstants.kWristOffset, 0);
        m_goal = m_setpoint;
    }
}
