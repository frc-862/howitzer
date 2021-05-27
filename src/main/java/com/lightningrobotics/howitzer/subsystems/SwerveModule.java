package com.lightningrobotics.howitzer.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.Constants.ModuleConstants;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {

    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private CANCoder canCoder;

    private SwerveModuleState desiredState;

    private double thinkingSpeed = 0d;

    // TODO set up and tune drive PID
    private final PIDController driveController = new PIDController(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 0.0);

    private final ProfiledPIDController turnController = new ProfiledPIDController(ModuleConstants.ANGLE_P, ModuleConstants.ANGLE_I, ModuleConstants.ANGLE_D, new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCEL));
    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.5, 0.5);

    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        // Configure CanCoder
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180; // Set the range to -180, 180 degrees
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees(); // Set the provided magnet offset
        canCoder.configAllSettings(canCoderConfiguration);

        // Set drive and anglemotors to brake mode
        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        //TODO verify this is the optimal unit
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        desiredState = new SwerveModuleState(0d, Rotation2d.fromDegrees(0d)); // Initialize desiredstate to 0
        //TODO Remove shuffleboard debug statements at some point
        Shuffleboard.getTab("Drivetrain").addString(("Desired State for " + angleMotor.getDeviceID()), () -> desiredState.toString());
        Shuffleboard.getTab("Drivetrain").addNumber(("Speed " + angleMotor.getDeviceID()), () -> thinkingSpeed);

    }

    /**
     * Gets the relative rotational position of the module
     * @return The relative rotational position of the angle motor in degrees
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()); // Since encoder is in abs mode, this should get the absolute value
                                                               // passing this to rotation should constrain to +/-180
    }

    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;

        Rotation2d currentRotation = getAngle();

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation); // Optimize the turning of a state

        //TODO replace this with a PID loop
        thinkingSpeed = (state.speedMetersPerSecond / DrivetrainConstants.MAX_SPEED); // convert meters per second to a range from -1 to 1
        driveMotor.set( thinkingSpeed * 0.3 ); // Limit speed to 30% (remove later)

        final double turnOutput = turnController.calculate(currentRotation.getRadians(), state.angle.getRadians()); // Get the optimal voltage output to turn to the specified rotation
        final double turnFeedForward = turnFF.calculate(turnController.getSetpoint().velocity); // Caculate the feedforward
        angleMotor.setVoltage(turnOutput + turnFeedForward);

    }
    /** 
     * @return      A state made up of the current speed and cancoder position
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorPosition(), new Rotation2d(canCoder.getAbsolutePosition()));
    }

}
