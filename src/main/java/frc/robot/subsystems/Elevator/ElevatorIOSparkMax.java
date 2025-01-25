package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;

public class ElevatorIOSparkMax implements ElevatorIO{
    private final SparkMax m_leftElevator;
    private final SparkMax m_rightElevator;
    private final SparkAbsoluteEncoder m_elevatorEncoder;
    private final SparkClosedLoopController m_elevatorController;
    private final SimpleMotorFeedforward m_pivotFeedforward = new SimpleMotorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV);
    private final Constraints m_pivotConstraints = new Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    public ElevatorIOSparkMax(){
        m_leftElevator = new SparkMax(HardwareConstants.kLeftElevatorCanId, MotorType.kBrushless);
        m_rightElevator = new SparkMax(HardwareConstants.kRightElevatorCanId, MotorType.kBrushless);

        m_elevatorEncoder = m_leftElevator.getAbsoluteEncoder();

        m_elevatorController = m_leftElevator.getClosedLoopController();

        m_leftElevator.configure(Configs.ElevatorConfig.m_leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightElevator.configure(Configs.ElevatorConfig.m_rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset goal to 0 on init
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.m_elevatorGoal = m_goal.position;
        inputs.m_elevatorPos = getPosition();
        inputs.m_elevatorCurrent = getCurrent();
    };

    @Override
    public void setGoal(double p_elevatorGoal){
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = new TrapezoidProfile.State(p_elevatorGoal, 0);
    }

    @Override
    public double getGoal(){
        return m_goal.position;
    }

    @Override
    public double getPosition(){
        return m_elevatorEncoder.getPosition();
    }

    @Override
    public double getCurrent(){
        return m_leftElevator.getOutputCurrent();
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_pivotConstraints).calculate(0.02, m_setpoint, m_goal);
        m_elevatorController.setReference(m_setpoint.position, ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, m_pivotFeedforward.calculate(m_setpoint.velocity));
    }
}
