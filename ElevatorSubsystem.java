package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMax;
import com.revrobotics.SparkMaxConfig;
import com.revrobotics.SparkClosedLoopController;
import com.revrobotics.SparkMax.ControlType;
import com.revrobotics.SparkMax.FeedbackSensor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Motor controller for elevator motor 1
    private SparkMax m_elevator1 = new SparkMax(ElevatorConstants.kElevator1CanId, MotorType.kBrushless);
    // Closed loop controller for elevator motor 1
    private SparkClosedLoopController m_elevator1Controller = m_elevator1.getClosedLoopController();
    // Encoder for elevator motor 1
    private RelativeEncoder m_elevator1Encoder = m_elevator1.getEncoder();

    // Motor controller for elevator motor 2
    private SparkMax m_elevator2 = new SparkMax(ElevatorConstants.kElevator2CanId, MotorType.kBrushless);
    // Closed loop controller for elevator motor 2
    private SparkClosedLoopController m_elevator2Controller = m_elevator2.getClosedLoopController();
    // Encoder for elevator motor 2
    private RelativeEncoder m_elevator2Encoder = m_elevator2.getEncoder();

    // Constants related to the elevator system
    public static final class ElevatorConstants {

        // CAN IDs for the elevator motors
        public static final int kElevator1CanId = 31;
        public static final int kElevator2CanId = 32;

        // Predefined positions for the elevator
        public static final double kHome = 0;
        public static final double kFeederStation = 14;
        public static final double kTravel = 0;

        // Positions for different levels
        public static final double kLevel0 = 4;
        public static final double kLevel1 = 18;
        public static final double kLevel2 = 58;
        public static final double kLevel3 = 103;
        public static final double kLevel4 = 130;
    }

    // Configuration settings for the elevator subsystem
    public class Configs {

        public static final class ElevatorSubsystem {

            // Configuration for elevator motor 1
            public static final SparkMaxConfig elevator1Config = new SparkMaxConfig();
            // Configuration for elevator motor 2
            public static final SparkMaxConfig elevator2Config = new SparkMaxConfig();

            static {
                // Configure elevator motor 1 settings
                elevator1Config
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50)
                        .voltageCompensation(12)
                        .inverted(true);

                // Configure closed loop settings for elevator motor 1
                elevator1Config.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(0.2)
                        .outputRange(-1, 1).maxMotion
                        .maxVelocity(4000)
                        .maxAcceleration(8000)
                        .allowedClosedLoopError(0.5);

                // Configure elevator motor 2 settings
                elevator2Config
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50)
                        .voltageCompensation(12)
                        .inverted(false);

                // Configure closed loop settings for elevator motor 2
                elevator2Config.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .p(0.2)
                        .outputRange(-1, 1).maxMotion
                        .maxVelocity(4000)
                        .maxAcceleration(8000)
                        .allowedClosedLoopError(0.5);
            }
        }
    }

    // Constructor for the ElevatorSub class
    public ElevatorSubsystem() {
        // Apply the configurations to the elevator motors
        m_elevator1.applyConfiguration(Configs.ElevatorSubsystem.elevator1Config);
        m_elevator2.applyConfiguration(Configs.ElevatorSubsystem.elevator2Config);
    }

    // This method moves the elevator to the specified setpoint using MaxMotion
    // MaxMotion is a trapezoidal profile and we can manipulate its PID, velocity
    // and acceleration in configs
    public void moveElevatorToPosition(double Setpoint) {
        // Set the reference position for elevator motor 1 using MaxMotion control
        m_elevator1Controller.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
        // Set the reference position for elevator motor 2 using MaxMotion control
        m_elevator2Controller.setReference(Setpoint, ControlType.kMAXMotionPositionControl);
    }

    // Get the position of Elevator Motor 1
    public double getElevator1Position() {
        // Return the current position of the encoder for elevator motor 1
        return m_elevator1Encoder.getPosition();
    }

    // Get Position of Elevator Motor 2
    public double getElevator2Position() {
        // Return the current position of the encoder for elevator motor 2
        return m_elevator2Encoder.getPosition();
    }

    // Reset the Elevator Encoders to 0
    public void resetElevatorEncoders() {
        // Set the position of the encoder for elevator motor 1 to 0
        m_elevator1Encoder.setPosition(0);
        // Set the position of the encoder for elevator motor 2 to 0
        m_elevator2Encoder.setPosition(0);
    }

    // Stop the Elevator Motors
    public void stopElevatorMotors() {
        // Stop the motor for elevator motor 1
        m_elevator1.stopMotor();
        // Stop the motor for elevator motor 2
        m_elevator2.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Display the current position of elevator motor 1 on the SmartDashboard
        SmartDashboard.putNumber("elevator 1 encoder: ", getElevator1Position());
        // Display the current position of elevator motor 2 on the SmartDashboard
        SmartDashboard.putNumber("elevator 2 encoder: ", getElevator2Position());
    }
}