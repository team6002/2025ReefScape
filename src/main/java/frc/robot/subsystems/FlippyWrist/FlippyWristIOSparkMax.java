package frc.robot.subsystems.FlippyWrist;

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
import frc.robot.Constants.FlippyWristConstants;
import frc.robot.Constants.HardwareConstants;

public class FlippyWristIOSparkMax implements FlippyWristIO{
    private final SparkMax m_flippyWristMotor;
    private final AbsoluteEncoder m_flippyWristEncoder;
    private final SparkClosedLoopController m_flippyWristController;
    private  ArmFeedforward m_flippyWristFeedforward = new ArmFeedforward(FlippyWristConstants.kS, FlippyWristConstants.kG, FlippyWristConstants.kV);
    private Constraints m_flippyWristConstraints = new Constraints(FlippyWristConstants.kMaxVel, FlippyWristConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public FlippyWristIOSparkMax(){
        //initialize motor
        m_flippyWristMotor = new SparkMax(HardwareConstants.kFlippyWristCanId, MotorType.kBrushless);

        //initialize PID controller
        m_flippyWristController = m_flippyWristMotor.getClosedLoopController();

        //initalize encoder
        m_flippyWristEncoder = m_flippyWristMotor.getAbsoluteEncoder();

        //apply config
        m_flippyWristMotor.configure(Configs.FlippyWristConfigs.m_flippyWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset reference in init
        m_setpoint = new TrapezoidProfile.State(getPosition() - FlippyWristConstants.kWristOffset, 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(FlippyWristIOInputs inputs){
        inputs.m_flippyWristGoal = Math.toDegrees(getGoal());
        inputs.m_flippyWristCurrent = getCurrent();
        inputs.m_flippyWristPosition = Math.toDegrees(getPosition());
        inputs.m_flippyWristInPosition = inPosition();
        inputs.m_flippyWristSetpoint = Math.toDegrees(getSetpoint());
    }

    @Override
    public void setGoal(double p_Goal){
        m_setpoint = new TrapezoidProfile.State(getPosition() - FlippyWristConstants.kWristOffset, 0);
        m_goal = new TrapezoidProfile.State(p_Goal - FlippyWristConstants.kWristOffset, 0);
    }

    @Override
    public double getPosition(){
        return m_flippyWristEncoder.getPosition()+FlippyWristConstants.kWristOffset;
    }

    @Override
    public double getCurrent(){
        return m_flippyWristMotor.getOutputCurrent();
    }

    @Override
    public double getGoal(){
        return m_goal.position + FlippyWristConstants.kWristOffset;
    }

    @Override 
    public double getSetpoint(){
        return m_setpoint.position + FlippyWristConstants.kWristOffset;
    }

    @Override
    public boolean inPosition(){
        return Math.abs(getPosition() - getGoal()) < FlippyWristConstants.kTolerance;
    }

    @Override
    public boolean inPosition(double p_position){
        return Math.abs(getPosition() - p_position) < FlippyWristConstants.kTolerance;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_flippyWristConstraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_flippyWristController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_flippyWristFeedforward.calculate(getPosition() + GlobalVariables.m_pivotAngle, m_setpoint.velocity));
    }

    @Override
    public void setConstraints(double velocity, double acceleration){
        m_flippyWristConstraints = new Constraints(velocity, acceleration);

    }
    @Override
    public void reset(){
        m_setpoint = new TrapezoidProfile.State(getPosition() - FlippyWristConstants.kWristOffset, 0);
        m_goal = m_setpoint;
        m_flippyWristController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_flippyWristFeedforward.calculate(getPosition() + GlobalVariables.m_pivotAngle, m_setpoint.velocity));
    }
}
