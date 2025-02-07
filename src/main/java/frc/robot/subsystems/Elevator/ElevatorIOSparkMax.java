package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.GlobalVariables;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;

public class ElevatorIOSparkMax implements ElevatorIO{
    private final SparkMax m_leftElevator;
    private final SparkMax m_rightElevator;
    private final RelativeEncoder m_elevatorEncoder;
    private final SparkClosedLoopController m_elevatorController;
    private final ArmFeedforward m_feedforward = new ArmFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
    private Constraints m_constraints = new Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel);
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    public ElevatorIOSparkMax(){
        m_leftElevator = new SparkMax(HardwareConstants.kLeftElevatorCanId, MotorType.kBrushless);
        m_rightElevator = new SparkMax(HardwareConstants.kRightElevatorCanId, MotorType.kBrushless);

        m_elevatorEncoder = m_rightElevator.getEncoder();

        m_elevatorController = m_rightElevator.getClosedLoopController();

        m_leftElevator.configure(Configs.ElevatorConfig.m_leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightElevator.configure(Configs.ElevatorConfig.m_rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset goal to 0 on init
        m_elevatorEncoder.setPosition(0);
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(ElevatorIoInputs inputs) {
        inputs.m_elevatorGoal = m_goal.position;
        inputs.m_elevatorPos = getPosition();
        inputs.m_elevatorCurrent = getCurrent();
        inputs.m_inPosition = inPosition();
    };

    @Override
    public void setGoal(double p_elevatorGoal){
        if(p_elevatorGoal < getPosition()){
            m_constraints = new Constraints(ElevatorConstants.kMaxVelDown, ElevatorConstants.kMaxAccelDown);
        }else{
            m_constraints = new Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel);
        }
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
        return m_rightElevator.getOutputCurrent();
    }

    @Override
    public void reset(){
        m_elevatorEncoder.setPosition(0);
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = m_setpoint;
    }

    @Override
    public double getSetpoint(){
        return m_setpoint.position;
    }

    @Override
    public boolean inPosition(){
        return Math.abs(getGoal() - getPosition()) < ElevatorConstants.kTolerance;
    }

    @Override
    public void PID(){
        var profile = new TrapezoidProfile(m_constraints).calculate(0.02, m_setpoint, m_goal);
        m_setpoint = profile;
        m_elevatorController.setReference(m_setpoint.position, ControlType.kPosition,
            ClosedLoopSlot.kSlot0, m_feedforward.calculate(GlobalVariables.m_pivotAngle - Math.toRadians(90), m_setpoint.velocity));
    }
}
