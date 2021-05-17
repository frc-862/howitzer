package com.lightningrobotics.howitzer;

import java.util.Arrays;

import com.lightningrobotics.howitzer.Constants.JoystickConstants;
import com.lightningrobotics.howitzer.commands.ModuleControl;
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
    
    private final XboxController controller = new XboxController(JoystickConstants.DRIVER_CONTROLLER);

    private static final Drivetrain drivetrain = new Drivetrain();

    private static SendableChooser<Command> chooser = new SendableChooser<>();

    

    @Override
    protected void configureAutonomousCommands() 
    {
        Autonomous.register("Forward", new TestPathCommand().getCommand(drivetrain, 
            Arrays.asList(new Pose2d(0d, 0d, new Rotation2d()), new Pose2d(1d, 0d, new Rotation2d()))));

        Autonomous.register("Backward", new TestPathCommand().getCommand(drivetrain, 
            Arrays.asList(new Pose2d(0d, 0d, new Rotation2d()), new Pose2d(-1d, 0d, new Rotation2d()))));
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
        Shuffleboard.getTab("Module Controller").add("Front Left",
            new ModuleControl(drivetrain, ModuleNumber.FRONT_LEFT_MOTOR, 0.2));

        Shuffleboard.getTab("Module Controller").add("Front Right",
            new ModuleControl(drivetrain, ModuleNumber.FRONT_RIGHT_MOTOR, 0.2));

        Shuffleboard.getTab("Module Controller").add("Back Left",
            new ModuleControl(drivetrain, ModuleNumber.BACK_LEFT_MOTOR, 0.2));

        Shuffleboard.getTab("Module Controller").add("Back Right",
            new ModuleControl(drivetrain, ModuleNumber.BACK_RIGHT_MOTOR, 0.2));
    }

    @Override
    protected void releaseDefaultCommands() {}
    
}
