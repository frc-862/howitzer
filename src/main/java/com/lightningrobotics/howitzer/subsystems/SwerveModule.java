package com.lightningrobotics.howitzer.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.Constants.ModuleConstants;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;

public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private CANCoder canCoder;

    public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

        angleTalonFXConfiguration.slot0.kP = ModuleConstants.ANGLE_P;
        angleTalonFXConfiguration.slot0.kI = ModuleConstants.ANGLE_I;
        angleTalonFXConfiguration.slot0.kD = ModuleConstants.ANGLE_D;

        // Use the CANCoder as the remote sensor for the primary TalonFX PID
        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleTalonFXConfiguration);

        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

        driveTalonFXConfiguration.slot0.kP = ModuleConstants.DRIVE_P;
        driveTalonFXConfiguration.slot0.kI = ModuleConstants.DRIVE_I;
        driveTalonFXConfiguration.slot0.kD = ModuleConstants.DRIVE_D;
        driveTalonFXConfiguration.slot0.kF = ModuleConstants.DRIVE_F;

        driveMotor.configAllSettings(driveTalonFXConfiguration);

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoder.configAllSettings(canCoderConfiguration);

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
        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // Find the difference between our current rotational position + our new
        // rotational position
        Rotation2d rotationDelta = state.angle.minus(currentRotation);

        // Find the new absolute position of the module based on the difference in rotation
        double deltaTicks = (rotationDelta.getDegrees() / 360) * ModuleConstants.TICKS_PER_REV_CANCODER;

        // Convert the CANCoder from it's position reading back to ticks
        double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
        double desiredTicks = currentTicks + deltaTicks;
        angleMotor.set(TalonFXControlMode.Position, desiredTicks);

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
        driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / DrivetrainConstants.MAX_SPEED);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorPosition(), new Rotation2d(canCoder.getAbsolutePosition()));
    }

}
