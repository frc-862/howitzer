package com.lightningrobotics.howitzer.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lightning.subsystems.IMU;

public class Drivetrain extends SubsystemBase {

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        DrivetrainConstants.FRONT_LEFT_POS,
        DrivetrainConstants.FRONT_RIGHT_POS,
        DrivetrainConstants.BACK_LEFT_POS,
        DrivetrainConstants.BACK_RIGHT_POS
    );

    private final IMU imu = IMU.pigeon(RobotMap.PIGEON);

    // TODO: Update these CAN device IDs to match your TalonFX + CANCoder device IDs
    // TODO: Update module offsets to match your CANCoder offsets
    private SwerveModule[] modules = new SwerveModule[] {
        new SwerveModule(new TalonFX(RobotMap.FRONT_LEFT_DRIVE_MOTOR),  new TalonFX(RobotMap.FRONT_LEFT_ANGLE_MOTOR),  new CANCoder(RobotMap.FRONT_LEFT_CANCODER),  Rotation2d.fromDegrees(0)), // Front Left
        new SwerveModule(new TalonFX(RobotMap.FRONT_RIGHT_DRIVE_MOTOR), new TalonFX(RobotMap.FRONT_RIGHT_ANGLE_MOTOR), new CANCoder(RobotMap.FRONT_RIGHT_CANCODER), Rotation2d.fromDegrees(0)), // Front Right
        new SwerveModule(new TalonFX(RobotMap.BACK_LEFT_DRIVE_MOTOR),   new TalonFX(RobotMap.BACK_LEFT_ANGLE_MOTOR),   new CANCoder(RobotMap.BACK_LEFT_CANCODER),   Rotation2d.fromDegrees(0)), // Back Left
        new SwerveModule(new TalonFX(RobotMap.BACK_RIGHT_DRIVE_MOTOR),  new TalonFX(RobotMap.BACK_RIGHT_ANGLE_MOTOR),  new CANCoder(RobotMap.BACK_RIGHT_CANCODER),  Rotation2d.fromDegrees(0)) // Back Right
    };

    public Drivetrain() {
        imu.reset();
    }

    /**
     * Method to drive the robot using joystick info.
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.getHeading())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DrivetrainConstants.MAX_SPEED);
        for (int i = 0; i < states.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setDesiredState(state);
        }
    }

}