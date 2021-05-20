package com.lightningrobotics.howitzer;

import java.util.Arrays;

import com.lightningrobotics.howitzer.Constants.JoystickConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;
import com.lightningrobotics.howitzer.commands.ModuleControl;
import com.lightningrobotics.howitzer.commands.MotorTest;
import com.lightningrobotics.howitzer.commands.SwerveDriveCommand;
import com.lightningrobotics.howitzer.commands.ModuleControl.ModuleNumber;
import com.lightningrobotics.howitzer.commands.auto.TestPathCommand;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lightning.LightningConfig;
import frc.lightning.LightningContainer;
import frc.lightning.auto.Autonomous;
import frc.lightning.subsystems.LightningDrivetrain;

public class RobotContainer extends LightningContainer {
    
    private static final XboxController controller = new XboxController(JoystickConstants.DRIVER_CONTROLLER);

    private static final Drivetrain drivetrain = new Drivetrain();

    

    @Override
    protected void configureAutonomousCommands() 
    {
        // Autonomous.register("Forward", new TestPathCommand().getCommand(drivetrain, 
        //     Arrays.asList(new Pose2d(0d, 0d, new Rotation2d()), new Pose2d(1d, 0d, new Rotation2d()))));

        // Autonomous.register("Backward", new TestPathCommand().getCommand(drivetrain, 
        //     Arrays.asList(new Pose2d(0d, 0d, new Rotation2d()), new Pose2d(-1d, 0d, new Rotation2d()))));
    }

    @Override
    protected void configureButtonBindings() {}

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, controller));
    }

    @Override
    protected void configureSystemTests() {}

    @Override
    public LightningConfig getConfig() {
        return null;
    }

    @Override
    public LightningDrivetrain getDrivetrain() {
        return null;
    }

    @Override
    protected void initializeDashboardCommands() {
        // Shuffleboard.getTab("Motors").add("Front Right Turn", new MotorTest(RobotMap.FRONT_RIGHT_ANGLE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Front Left Turn", new MotorTest(RobotMap.FRONT_LEFT_ANGLE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Back Right Turn", new MotorTest(RobotMap.BACK_RIGHT_ANGLE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Back Left Turn", new MotorTest(RobotMap.BACK_LEFT_ANGLE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Front Right Drive", new MotorTest(RobotMap.FRONT_RIGHT_DRIVE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Front Left Drive", new MotorTest(RobotMap.FRONT_LEFT_DRIVE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Back Right Drive", new MotorTest(RobotMap.BACK_RIGHT_DRIVE_MOTOR));
        // Shuffleboard.getTab("Motors").add("Back Left Drive", new MotorTest(RobotMap.BACK_LEFT_DRIVE_MOTOR));
        Shuffleboard.getTab("Module Controller").add("Front Left",
            new ModuleControl(drivetrain, ModuleNumber.FRONT_LEFT_MOTOR, 0.15));

        Shuffleboard.getTab("Module Controller").add("Front Right",
            new ModuleControl(drivetrain, ModuleNumber.FRONT_RIGHT_MOTOR, 0.15));

        Shuffleboard.getTab("Module Controller").add("Back Left",
            new ModuleControl(drivetrain, ModuleNumber.BACK_LEFT_MOTOR, 0.15));

        Shuffleboard.getTab("Module Controller").add("Back Right",
            new ModuleControl(drivetrain, ModuleNumber.BACK_RIGHT_MOTOR, 0.15));
    }

    @Override
    protected void releaseDefaultCommands() {}
    
}
