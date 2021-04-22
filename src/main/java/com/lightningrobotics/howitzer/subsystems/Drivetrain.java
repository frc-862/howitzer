package com.lightningrobotics.howitzer.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;
import com.lightningrobotics.howitzer.Constants.ModuleConstants;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
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
    private final PIDController ffController = new PIDController(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D);
    //private final PIDController xPidController = new PIDController(kp, ki, kd);
    //private final PIDController yPidController = new PIDController(kp, ki, kd);

    // TODO: Update these CAN device IDs to match your TalonFX + CANCoder device IDs
    // TODO: Update module offsets to match your CANCoder offsets
    private SwerveModule[] modules = new SwerveModule[] {
        new SwerveModule(new TalonFX(RobotMap.FRONT_LEFT_DRIVE_MOTOR),  new TalonFX(RobotMap.FRONT_LEFT_ANGLE_MOTOR),  new CANCoder(RobotMap.FRONT_LEFT_CANCODER),  Rotation2d.fromDegrees(0)), // Front Left
        new SwerveModule(new TalonFX(RobotMap.FRONT_RIGHT_DRIVE_MOTOR), new TalonFX(RobotMap.FRONT_RIGHT_ANGLE_MOTOR), new CANCoder(RobotMap.FRONT_RIGHT_CANCODER), Rotation2d.fromDegrees(0)), // Front Right
        new SwerveModule(new TalonFX(RobotMap.BACK_LEFT_DRIVE_MOTOR),   new TalonFX(RobotMap.BACK_LEFT_ANGLE_MOTOR),   new CANCoder(RobotMap.BACK_LEFT_CANCODER),   Rotation2d.fromDegrees(0)), // Back Left
        new SwerveModule(new TalonFX(RobotMap.BACK_RIGHT_DRIVE_MOTOR),  new TalonFX(RobotMap.BACK_RIGHT_ANGLE_MOTOR),  new CANCoder(RobotMap.BACK_RIGHT_CANCODER),  Rotation2d.fromDegrees(0)) // Back Right
    };

    private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(kinematics, imu.getHeading());

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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DrivetrainConstants.MAX_SPEED);
        modules[0].setDesiredState(desiredStates[0]);
        modules[1].setDesiredState(desiredStates[1]);
        modules[2].setDesiredState(desiredStates[2]);
        modules[3].setDesiredState(desiredStates[3]);
    }

    public void updateOdometry() {
        m_odometry.update(
            imu.getHeading(),
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState());
    }

    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }

    public double getMaxVelocity() {
        return DrivetrainConstants.MAX_SPEED;
    }

    public double getMaxAcceleration() {
        return 3d;
    }

    public PIDController getFeedforward() {
        return ffController;
    }

    public PIDController getXPidController() {
        return null;
    }

    public PIDController getYPidController() {
        return null;
    }

    public ProfiledPIDController getThetaPidController() {
        return null;
    }

    public void stop() {
        drive(0d, 0d, imu.getHeading().getDegrees(), false);
    }
}
