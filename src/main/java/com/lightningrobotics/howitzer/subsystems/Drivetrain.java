package com.lightningrobotics.howitzer.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;
import com.lightningrobotics.howitzer.Constants.ModuleConstants;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // private final PIDController xPidController = new PIDController(kp, ki, kd);
    // private final PIDController yPidController = new PIDController(kp, ki, kd);

    public final SwerveModule[] modules;
    private final SwerveDriveOdometry odometry;
    private final SwerveDriveKinematics kinematics;

    // private final IMU imu;
    private final PigeonIMU imu;
    private final PIDController ffController;

    private double[] ypr = new double[3];

    public Drivetrain() {

        modules = new SwerveModule[] {
                new SwerveModule(new WPI_TalonFX(RobotMap.FRONT_LEFT_DRIVE_MOTOR),
                        new WPI_TalonFX(RobotMap.FRONT_LEFT_ANGLE_MOTOR), new CANCoder(RobotMap.FRONT_LEFT_CANCODER),
                        Rotation2d.fromDegrees(-95.09765625)), // Front Left
                new SwerveModule(new WPI_TalonFX(RobotMap.FRONT_RIGHT_DRIVE_MOTOR),
                        new WPI_TalonFX(RobotMap.FRONT_RIGHT_ANGLE_MOTOR), new CANCoder(RobotMap.FRONT_RIGHT_CANCODER),
                        Rotation2d.fromDegrees(-12.744140625)), // Front Right
                new SwerveModule(new WPI_TalonFX(RobotMap.BACK_LEFT_DRIVE_MOTOR),
                        new WPI_TalonFX(RobotMap.BACK_LEFT_ANGLE_MOTOR), new CANCoder(RobotMap.BACK_LEFT_CANCODER),
                        Rotation2d.fromDegrees(30.673828125)), // Back Left
                new SwerveModule(new WPI_TalonFX(RobotMap.BACK_RIGHT_DRIVE_MOTOR),
                        new WPI_TalonFX(RobotMap.BACK_RIGHT_ANGLE_MOTOR), new CANCoder(RobotMap.BACK_RIGHT_CANCODER),
                        Rotation2d.fromDegrees(119.00390625)) // Back Right
        };

        imu = new PigeonIMU(RobotMap.PIGEON);
        imu.configFactoryDefault();
        imu.setYaw(0d);
        imu.setAccumZAngle(0d);
        // imu = IMU.pigeon(RobotMap.PIGEON);
        // imu.reset();

        Shuffleboard.getTab("Drivetrain").addNumber("Heading", () -> getHeading().getDegrees());

        ffController = new PIDController(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D);

        kinematics = new SwerveDriveKinematics(DrivetrainConstants.FRONT_LEFT_POS, DrivetrainConstants.FRONT_RIGHT_POS,
                DrivetrainConstants.BACK_LEFT_POS, DrivetrainConstants.BACK_RIGHT_POS);

        odometry = new SwerveDriveOdometry(kinematics, getHeading());

        Shuffleboard.getTab("Drivetrain").addNumber("Front Left CANCoder", () -> modules[0].getAngle().getDegrees());
        Shuffleboard.getTab("Drivetrain").addNumber("Front Right CANCoder", () -> modules[1].getAngle().getDegrees());
        Shuffleboard.getTab("Drivetrain").addNumber("Back Left CANCoder", () -> modules[2].getAngle().getDegrees());
        Shuffleboard.getTab("Drivetrain").addNumber("Back Right CANCoder", () -> modules[3].getAngle().getDegrees());

    }

    @Override
    public void periodic() {
        super.periodic();
        imu.getYawPitchRoll(ypr);
    }

    public Rotation2d getHeading() {
        double heading = ypr[0];
        double sign = Math.signum(heading);
        double filteredRot = sign * (((Math.abs(heading) + 180) % 360) - 180); 
        return Rotation2d.fromDegrees(filteredRot);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        // SwerveModule module = modules[1];
        // SwerveModuleState state = states[1];
        // module.setDesiredState(state);

        for (int i = 0; i < states.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setDesiredState(state);
        }
    }

    /**
     * Method to drive the robot using joystick info.
     * 
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()));
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DrivetrainConstants.MAX_SPEED);
        // modules[0].setDesiredState(states[0]);
        // modules[1].setDesiredState(states[1]);
        // modules[2].setDesiredState(states[2]);
        // modules[3].setDesiredState(states[3]);
        setModuleStates(states);
    }

    public void updateOdometry() {
        odometry.update(getHeading(), modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
    }

    public Pose2d getOdometryPose2d() {
        updateOdometry();
        return odometry.getPoseMeters();
    }

    public Rotation2d getOdometryRotation2d() {
        updateOdometry();
        return odometry.getPoseMeters().getRotation();
    }

    public SwerveDriveKinematics getKinematics() {
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
        drive(0d, 0d, getHeading().getDegrees());
    }
}
