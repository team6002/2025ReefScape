package frc.robot.subsystems.SpinnyWrist;

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
import frc.robot.Constants.SpinnyWristConstants;
import frc.robot.Constants.HardwareConstants;

public class SpinnyWristIOSparkMax implements SpinnyWristIO{
    private final SparkMax m_spinnyWristMotor;
    private final AbsoluteEncoder m_spinnyWristEncoder;
    private final SparkClosedLoopController m_spinnyWristController;
    private  ArmFeedforward m_spinnyWristFeedforward = new ArmFeedforward(SpinnyWristConstants.kS, SpinnyWristConstants.kG, SpinnyWristConstants.kV);
    private Constraints m_spinnyWristConstraints = new Constraints(SpinnyWristConstants.kMaxVel, SpinnyWristConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public SpinnyWristIOSparkMax(){
        //initialize motor
        m_spinnyWristMotor = new SparkMax(HardwareConstants.kSpinnyWristCanId, MotorType.kBrushless);

        //initialize PID controller
        m_spinnyWristController = m_spinnyWristMotor.getClosedLoopController();

        //initalize encoder
        m_spinnyWristEncoder = m_spinnyWristMotor.getAbsoluteEncoder();

        //apply config
        m_spinnyWristMotor.configure(Configs.SpinnyWristConfigs.m_spinnyWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset reference in init
        m_setpoint = new TrapezoidProfile.State(getPosition() - SpinnyWristConstants.kWristOffset, 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(SpinnyWristIOInputs inputs){
        inputs.m_spinnyWristGoal = Math.toDegrees(getGoal());
        inputs.m_spinnyWristCurrent = getCurrent();
        inputs.m_spinnyWristPosition = Math.toDegrees(getPosition());
        inputs.m_spinnyWristInPosition = inPosition();
        inputs.m_spinnyWristSetpoint = Math.toDegrees(getSetpoint());
    }

    @Override
    public void setGoal(double p_goal){
        m_setpoint = new TrapezoidProfile.State(getPosition() - SpinnyWristConstants.kWristOffset, 0);
        m_goal = new TrapezoidProfile.State(p_goal - SpinnyWristConstants.kWristOffset, 0);
    }

    @Override
    public double getPosition(){
        return m_spinnyWristEncoder.getPosition()+SpinnyWristConstants.kWristOffset;
    }

    @Override
    public double getCurrent(){
        return m_spinnyWristMotor.getOutputCurrent();
    }

    @Override
    public double getGoal(){
        return m_goal.position + SpinnyWristConstants.kWristOffset;
    }

    @Override 
    public double getSetpoint(){
        return m_setpoint.position + SpinnyWristConstants.kWristOffset;
    }

    @Override
    public boolean inPosition(){
        return Math.abs(getPosition() - getGoal()) < SpinnyWristConstants.kTolerance;
    }

    @Override
    public boolean inPosition(double p_position){
        return Math.abs(getPosition() - p_position) < SpinnyWristConstants.kTolerance;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_spinnyWristConstraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_spinnyWristController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_spinnyWristFeedforward.calculate(getPosition() + GlobalVariables.m_pivotAngle, m_setpoint.velocity));
    }

    @Override
    public void setConstraints(double velocity, double acceleration){
        m_spinnyWristConstraints = new Constraints(velocity, acceleration);

    }
    @Override
    public void reset(){
        m_setpoint = new TrapezoidProfile.State(getPosition() - SpinnyWristConstants.kWristOffset, 0);
        m_goal = m_setpoint;
        m_spinnyWristController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_spinnyWristFeedforward.calculate(getPosition() + GlobalVariables.m_pivotAngle, m_setpoint.velocity));
    }
}
