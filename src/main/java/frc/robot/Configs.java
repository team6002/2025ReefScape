package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WinchConstants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = (ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction)*(ModuleConstants.kXFactor);
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 0;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12)
                    .smartCurrentLimit(90)
                    .inverted(false);
            drivingConfig.encoder
                    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor) // meters
                    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)// meters per second
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2); 
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD,ClosedLoopSlot.kSlot0)
                    .pid(ModuleConstants.kAutoP, ModuleConstants.kAutoI, ModuleConstants.kAutoD,ClosedLoopSlot.kSlot1)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            drivingConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(30);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    .averageDepth(2);
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
            turningConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);
        }
    }

    public static final class ElevatorConfig{
        public static final SparkMaxConfig m_leftElevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_rightElevatorConfig = new SparkMaxConfig();

        static {
                m_leftElevatorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorConstants.kLeftInverted)
                        .follow(HardwareConstants.kRightElevatorCanId, true)
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_leftElevatorConfig.closedLoop
                        .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
                m_leftElevatorConfig.encoder
                        .uvwAverageDepth(2)
                        .uvwMeasurementPeriod(10)
                        .positionConversionFactor(ElevatorConstants.kConversionFactor);
                m_leftElevatorConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);

                m_rightElevatorConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorConstants.kRightInverted)
                        .disableFollowerMode()
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_rightElevatorConfig.closedLoop
                        .pidf(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
                m_rightElevatorConfig.encoder
                        .uvwAverageDepth(2)
                        .uvwMeasurementPeriod(10)
                        .positionConversionFactor(ElevatorConstants.kConversionFactor);
                m_rightElevatorConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }

        public static final class WristConfigs{
                public static final SparkMaxConfig m_wristConfig = new SparkMaxConfig();
                static {
                        m_wristConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(WristConstants.kWristInverted)
                                .smartCurrentLimit(40)
                                .disableFollowerMode()
                                .voltageCompensation(12.0);
                        m_wristConfig.encoder
                                .positionConversionFactor(WristConstants.kConverstionFactor)
                                .uvwAverageDepth(2)
                                .uvwMeasurementPeriod(10);
                        m_wristConfig.absoluteEncoder
                                .inverted(true)
                                .positionConversionFactor(WristConstants.kConverstionFactor)
                                .velocityConversionFactor(WristConstants.kConverstionFactor/60)
                                .averageDepth(2);
                        m_wristConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput)
                                .pidf(WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.kFF)
                                .positionWrappingInputRange(Math.toRadians(0), WristConstants.kConverstionFactor)
                                .positionWrappingEnabled(true);
                        m_wristConfig.limitSwitch
                                .forwardLimitSwitchEnabled(false)
                                .reverseLimitSwitchEnabled(false);
                }
        }
    public static final class ElevatorPivotConfig{
        public static final SparkMaxConfig m_leftPivotConfig = new SparkMaxConfig();
        public static final SparkMaxConfig m_rightPivotConfig = new SparkMaxConfig();

        static{
                m_leftPivotConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(PivotConstants.kLeftInverted)
                        .voltageCompensation(12.0)
                        .follow(HardwareConstants.kRightPivotCanId,true)
                        .smartCurrentLimit(40);
                m_leftPivotConfig.closedLoop
                        .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput);
                m_leftPivotConfig.encoder
                        .positionConversionFactor(PivotConstants.kConversionFactor)
                        .uvwMeasurementPeriod(10)
                        .uvwAverageDepth(2); 
                m_leftPivotConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);

                m_rightPivotConfig
                        .idleMode(IdleMode.kBrake)//Brake
                        .inverted(PivotConstants.kRightInverted)
                        .voltageCompensation(12.0)
                        .disableFollowerMode()
                        .smartCurrentLimit(40);
                m_rightPivotConfig.closedLoop
                        .pid(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD)
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .outputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput);
                m_rightPivotConfig.absoluteEncoder
                        .averageDepth(2)
                        .positionConversionFactor(PivotConstants.kConversionFactor);
                m_rightPivotConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }   
    
    public static final class CoralHolderConfig{
        public static final SparkMaxConfig m_intakeConfig = new SparkMaxConfig();

        static{
                m_intakeConfig
                        .disableFollowerMode()
                        .idleMode(IdleMode.kBrake)
                        .inverted(CoralHolderConstants.kCoralHolderInverted)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12.0);
                m_intakeConfig.encoder
                        .quadratureAverageDepth(2)
                        .quadratureMeasurementPeriod(10);
                m_intakeConfig.closedLoop
                        .pidf(CoralHolderConstants.kP, CoralHolderConstants.kI, CoralHolderConstants.kD, CoralHolderConstants.kFF)
                        .outputRange(CoralHolderConstants.kMinOutput, CoralHolderConstants.kMaxOutput)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                m_intakeConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }

    public static final class WinchConfig{
        public static final SparkMaxConfig m_WinchConfig = new SparkMaxConfig();

        static{
                m_WinchConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(false)
                        .disableFollowerMode()
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_WinchConfig.closedLoop
                        .pidf(WinchConstants.kP, WinchConstants.kI, WinchConstants.kD, WinchConstants.kFF)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(AlgaeConstants.kMinOutput, AlgaeConstants.kMaxOutput);
                m_WinchConfig.encoder
                        .uvwAverageDepth(2)
                        .uvwMeasurementPeriod(10)
                        .positionConversionFactor(1);
                m_WinchConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }

    public static final class AlgaeConfig{
        public static final SparkMaxConfig m_AlgaeConfig = new SparkMaxConfig();

        static{
                m_AlgaeConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(false)
                        .disableFollowerMode()
                        .voltageCompensation(12.0)
                        .smartCurrentLimit(40);
                m_AlgaeConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .outputRange(WinchConstants.kMinOutput, WinchConstants.kMaxOutput);
                m_AlgaeConfig.encoder
                        .uvwAverageDepth(2)
                        .uvwMeasurementPeriod(10)
                        .positionConversionFactor(360)
                        .velocityConversionFactor(1);
                m_AlgaeConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }
}
