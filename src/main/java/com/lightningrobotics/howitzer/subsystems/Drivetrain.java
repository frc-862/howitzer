package com.lightningrobotics.howitzer.subsystems;

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
        // Creating a list of our swerve modules 
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

        // Configuring our pigeon 
        imu = new PigeonIMU(RobotMap.PIGEON);
        imu.configFactoryDefault(); // set the IMU to factory default settings
        imu.setYaw(0d); // Set yaw (heading, about the z axis) to 0
        imu.setAccumZAngle(0d); // reset accumulated yaw (about the z axis) to 0

        // TODO for debuging, this could be removed later
        // Adding our heading to the shuffleboard
        Shuffleboard.getTab("Drivetrain").addNumber("Heading", () -> getHeading().getDegrees());

        // Creating PIDController object and setting our PID values
        ffController = new PIDController(ModuleConstants.DRIVE_P, ModuleConstants.DRIVE_I, ModuleConstants.DRIVE_D);

        // Creating kinematics object and setting the postion of the wheels relative to the center of the robot
        kinematics = new SwerveDriveKinematics(DrivetrainConstants.FRONT_LEFT_POS, DrivetrainConstants.FRONT_RIGHT_POS,
                DrivetrainConstants.BACK_LEFT_POS, DrivetrainConstants.BACK_RIGHT_POS);

        // Creating a swerveDriveOdometry object and setting kinematics and current heading  
        odometry = new SwerveDriveOdometry(kinematics, getHeading());

        // TODO for debuging, this could be removed later 
        // Adding swerve module angles to shuffleboard tabs 
        Shuffleboard.getTab("Drivetrain").addNumber("Front Left CANCoder", () -> modules[0].getAngle().getDegrees());
        Shuffleboard.getTab("Drivetrain").addNumber("Front Right CANCoder", () -> modules[1].getAngle().getDegrees());
        Shuffleboard.getTab("Drivetrain").addNumber("Back Left CANCoder", () -> modules[2].getAngle().getDegrees());
        Shuffleboard.getTab("Drivetrain").addNumber("Back Right CANCoder", () -> modules[3].getAngle().getDegrees());

    }

    @Override
    public void periodic() {
        super.periodic();
        imu.getYawPitchRoll(ypr); // update our yaw, pitch, and roll regularly
    }

    /** Get the current heading constrained to a range of -180 to 180 degrees */
    public Rotation2d getHeading() {
        double heading = ypr[0]; // getting yaw (orientation)
        double sign = Math.signum(heading); // get the sign of the current heading (returns 1 if positive, -1 if negative)
        double filteredRot = sign * (((Math.abs(heading) + 180) % 360) - 180); //  constrain the angle to be within -180 and 180 degrees
        return Rotation2d.fromDegrees(filteredRot); // return the angle in the form of a rotation2d object
    }

    /** 
     * Iterate through the provided states and apply each to the corresponding swerve module
     * @param states    A list containing states corresponding to each of the swerve modules
    */
    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) { // iterate through each item in the list of states
            SwerveModule module = modules[i]; // set module to be the selected module from the list
            SwerveModuleState state = states[i]; // set state to be the selected module from the list
            module.setDesiredState(state); // Set the desired state of the selected module to the selected state
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
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading())); // Get chassis speeds and convert them to swerve module states
        SwerveDriveKinematics.normalizeWheelSpeeds(states, DrivetrainConstants.MAX_SPEED); // Prevent motors from spinning over the max speed
        setModuleStates(states); //set all of the module states
    }

    /**  update the odometry value
    */
    public void updateOdometry() {
        odometry.update(getHeading(), modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()); 
    }


    // Accesser functions
    public Pose2d getOdometryPose2d() {
        updateOdometry(); // First, update the odometry
        return odometry.getPoseMeters(); // return the odometry position in meters
    }

    public Rotation2d getOdometryRotation2d() {
        updateOdometry(); // set module to be the selected module from the list
        return odometry.getPoseMeters().getRotation(); // return the rotation as a rotation2d
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getMaxVelocity() {
        return DrivetrainConstants.MAX_SPEED;
    }

    public double getMaxAcceleration() {
        return DrivetrainConstants.MAX_ACCEL;
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

    /** set the x and y speeds to 0
     * rotation is set to the current rotation
     */
    public void stop() {
        drive(0d, 0d, getHeading().getDegrees());
    }
}
