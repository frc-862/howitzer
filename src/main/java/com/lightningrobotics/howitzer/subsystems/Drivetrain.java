package com.lightningrobotics.howitzer.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.lightningrobotics.howitzer.Constants.ModuleConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;
import com.lightningrobotics.howitzer.Constants.Wheelbase;
import com.lightningrobotics.howitzer.util.SwerveKinematics;
import com.lightningrobotics.howitzer.util.DrivetrainSpeed;
import com.lightningrobotics.howitzer.util.SwerveModuleState;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    public enum Modules {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);
        private int idx;
        Modules(int idx) {
            this.idx = idx;
        }
        public int getIdx() {
            return this.idx;
        }
    }

    public final SwerveModule[] modules;

    private final SwerveKinematics kinematics;

    private SwerveModuleState[] states;

    private DrivetrainSpeed speed;

    public Drivetrain() {

        // Build modules
        modules = new SwerveModule[] {
            makeSwerveModule(
                    Modules.FRONT_LEFT,
                    RobotMap.FRONT_LEFT_DRIVE_MOTOR,
                    RobotMap.FRONT_LEFT_ANGLE_MOTOR,
                    RobotMap.FRONT_LEFT_CANCODER,
                    Rotation2d.fromDegrees(-95.09765625)),
                makeSwerveModule(
                        Modules.FRONT_RIGHT,
                        RobotMap.FRONT_RIGHT_DRIVE_MOTOR,
                        RobotMap.FRONT_RIGHT_ANGLE_MOTOR,
                        RobotMap.FRONT_RIGHT_CANCODER,
                        Rotation2d.fromDegrees(-12.744140625)),
                makeSwerveModule(
                        Modules.BACK_LEFT,
                        RobotMap.BACK_LEFT_DRIVE_MOTOR,
                        RobotMap.BACK_LEFT_ANGLE_MOTOR,
                        RobotMap.BACK_LEFT_CANCODER,
                        Rotation2d.fromDegrees(30.673828125)),
                makeSwerveModule(
                        Modules.BACK_RIGHT,
                        RobotMap.BACK_RIGHT_DRIVE_MOTOR,
                        RobotMap.BACK_RIGHT_ANGLE_MOTOR,
                        RobotMap.BACK_RIGHT_CANCODER,
                        Rotation2d.fromDegrees(119.00390625))
        };

        // Set up kinematics
        kinematics = new SwerveKinematics(Wheelbase.W, Wheelbase.L);

        // Initialize empty states
        states = new SwerveModuleState[]{
            new SwerveModuleState(0d, modules[Modules.FRONT_LEFT.getIdx()].getModuleAngle()),
                new SwerveModuleState(0d, modules[Modules.FRONT_RIGHT.getIdx()].getModuleAngle()),
                new SwerveModuleState(0d, modules[Modules.BACK_LEFT.getIdx()].getModuleAngle()),
                new SwerveModuleState(0d, modules[Modules.BACK_RIGHT.getIdx()].getModuleAngle())
        };

        // Initialize zero drive speed
        speed = new DrivetrainSpeed();

        // Put some data on shuffleboard
        var tab = Shuffleboard.getTab("Swerve Module States");

        tab.addString("FL Real", () -> modules[Modules.FRONT_LEFT.getIdx()].getState().toString());
        tab.addString("FL Target", () -> states[Modules.FRONT_LEFT.getIdx()].toString());

        tab.addString("FR Real", () -> modules[Modules.FRONT_RIGHT.getIdx()].getState().toString());
        tab.addString("FR Target", () -> states[Modules.FRONT_RIGHT.getIdx()].toString());

        tab.addString("BL Real", () -> modules[Modules.BACK_LEFT.getIdx()].getState().toString());
        tab.addString("BL Target", () -> states[Modules.BACK_LEFT.getIdx()].toString());

        tab.addString("BR Real", () -> modules[Modules.BACK_RIGHT.getIdx()].getState().toString());
        tab.addString("BR Target", () -> states[Modules.BACK_RIGHT.getIdx()].toString());

        tab.addString("Target Speed", () -> speed.toString());
        tab.addString("Real Speed", () -> kinematics.forward(getStates()).toString());

    }

    public void setModuleStates(SwerveModuleState[] states) {
        this.states = states;
        for (var i = 0 ; i < states.length ; ++i) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setState(state);
        }
    }

    public void drive(DrivetrainSpeed speed) {
        this.speed = speed;
        setModuleStates(kinematics.inverse(speed));
    }

    public void stop() {
        this.drive(new DrivetrainSpeed());
    }

    private SwerveModule makeSwerveModule(Modules module, int driveID, int angleID, int encoderID, Rotation2d offset) {

        // Set Up Drive Motor
        WPI_TalonFX driveMotor = new WPI_TalonFX(driveID);
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Set Up Angle Motor
        WPI_TalonFX angleMotor = new WPI_TalonFX(angleID);
        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setInverted(true);

        // Set Up Encoder
        CANCoder canCoder = new CANCoder(encoderID);
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoderConfiguration.sensorDirection = true;
        canCoder.configAllSettings(canCoderConfiguration);

        // Build Module
        return new SwerveModule(
                driveMotor,
                angleMotor,
                () -> Rotation2d.fromDegrees(canCoder.getAbsolutePosition()),
                () -> (((double) driveMotor.getSelectedSensorVelocity() * 10) / (2048.0 * Wheelbase.GEARING)) * Wheelbase.WHEEL_CIRCUMFERENCE, // m/s
                ModuleConstants.DRIVE_CONTROLLER,
                ModuleConstants.AZIMUTH_CONTROLLER
                );

    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{
            modules[Modules.FRONT_LEFT.getIdx()].getState(),
                modules[Modules.FRONT_RIGHT.getIdx()].getState(),
                modules[Modules.BACK_LEFT.getIdx()].getState(),
                modules[Modules.BACK_RIGHT.getIdx()].getState()
        };
    }

    public SwerveKinematics getKinematics() {
        return this.kinematics;
    }

}

