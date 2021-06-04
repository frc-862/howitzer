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

    private final PIDController driveController = new PIDController(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D);

    private final ProfiledPIDController turnController = new ProfiledPIDController(ModuleConstants.ANGLE_P, ModuleConstants.ANGLE_I, ModuleConstants.ANGLE_D, new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_ANGULAR_SPEED, DrivetrainConstants.MAX_ANGULAR_ACCEL));

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.0, 0.0);
    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0.5, 0.5);

    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoder.configAllSettings(canCoderConfiguration);

        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        turnController.enableContinuousInput(-Math.PI, Math.PI);

        desiredState = new SwerveModuleState(0d, Rotation2d.fromDegrees(0d));
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

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // double currentSpeedMetersPerSecond = 0;
        // final double driveOutput = driveController.calculate(currentSpeedMetersPerSecond, state.speedMetersPerSecond);
        // // final double driveOutput = driveController.calculate(0d, 0d);
        // final double driveFeedForward = driveFF.calculate(state.speedMetersPerSecond);
        // driveMotor.setVoltage(driveOutput + driveFeedForward);
        thinkingSpeed = (state.speedMetersPerSecond / DrivetrainConstants.MAX_SPEED); // METERS PER SECOND
        driveMotor.set( thinkingSpeed * 0.5 );

        final double turnOutput = turnController.calculate(currentRotation.getRadians(), state.angle.getRadians());
        final double turnFeedForward = turnFF.calculate(turnController.getSetpoint().velocity);
        angleMotor.setVoltage(turnOutput + turnFeedForward);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorPosition(), new Rotation2d(canCoder.getAbsolutePosition()));
    }

}
