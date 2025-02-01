package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.WristConstants;
import frc.robot.Constants.CoralHolderConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPivotConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig RightDrivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 0;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .voltageCompensation(12)
                    .smartCurrentLimit(40)
                    .inverted(false);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0)// meters per second
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2); 
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            drivingConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);

                RightDrivingConfig
                        .idleMode(IdleMode.kBrake)
                        .voltageCompensation(12)
                        .smartCurrentLimit(40)
                        .inverted(true);
                RightDrivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0)// meters per second
                        .uvwMeasurementPeriod(10)
                        .uvwAverageDepth(2); 
                RightDrivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
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
                                .positionConversionFactor(360)
                                .uvwAverageDepth(2)
                                .uvwMeasurementPeriod(10);
                        m_wristConfig.absoluteEncoder
                                .inverted(true)
                                .positionConversionFactor(360)
                                .velocityConversionFactor(6)
                                .averageDepth(2);
                        m_wristConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .outputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput)
                                .pidf(WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.kFF)
                                .positionWrappingInputRange(0, 360)
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
                        .inverted(ElevatorPivotConstants.kLeftInverted)
                        .voltageCompensation(12.0)
                        .follow(HardwareConstants.kRightPivotCanId,true)
                        .smartCurrentLimit(40);
                m_leftPivotConfig.closedLoop
                        .pid(ElevatorPivotConstants.kP, ElevatorPivotConstants.kI, ElevatorPivotConstants.kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange(ElevatorPivotConstants.kMinOutput, ElevatorPivotConstants.kMaxOutput);
                m_leftPivotConfig.encoder
                        .positionConversionFactor(360)
                        .uvwMeasurementPeriod(10)
                        .uvwAverageDepth(2); 
                m_leftPivotConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);

                m_rightPivotConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(ElevatorPivotConstants.kRightInverted)
                        .voltageCompensation(12.0)
                        .disableFollowerMode()
                        .smartCurrentLimit(40);
                m_rightPivotConfig.closedLoop
                        .pid(ElevatorPivotConstants.kP, ElevatorPivotConstants.kI, ElevatorPivotConstants.kD)
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .outputRange(ElevatorPivotConstants.kMinOutput, ElevatorPivotConstants.kMaxOutput);
                m_rightPivotConfig.absoluteEncoder
                        .averageDepth(2)
                        .positionConversionFactor(360);
                m_rightPivotConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }   
    
    public static final class CoralHolderConfig{
        public static final SparkMaxConfig m_coralHolderConfig = new SparkMaxConfig();

        static{
                m_coralHolderConfig
                        .disableFollowerMode()
                        .idleMode(IdleMode.kCoast)
                        .inverted(CoralHolderConstants.kCoralHolderInverted)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12.0);
                m_coralHolderConfig.encoder
                        .quadratureAverageDepth(2)
                        .quadratureMeasurementPeriod(10);
                m_coralHolderConfig.closedLoop
                        .pidf(CoralHolderConstants.kP, CoralHolderConstants.kI, CoralHolderConstants.kD, CoralHolderConstants.kFF)
                        .outputRange(CoralHolderConstants.kMinOutput, CoralHolderConstants.kMaxOutput)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                m_coralHolderConfig.limitSwitch
                        .forwardLimitSwitchEnabled(false)
                        .reverseLimitSwitchEnabled(false);
        }
    }
}
