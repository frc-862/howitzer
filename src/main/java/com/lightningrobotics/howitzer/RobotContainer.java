package com.lightningrobotics.howitzer;

import com.lightningrobotics.howitzer.Constants.JoystickConstants;
import com.lightningrobotics.howitzer.commands.ModuleControl;
import com.lightningrobotics.howitzer.commands.SwerveDriveCommand;
import com.lightningrobotics.howitzer.commands.ModuleControl.ModuleNumber;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lightning.LightningConfig;
import frc.lightning.LightningContainer;
import frc.lightning.subsystems.LightningDrivetrain;

public class RobotContainer extends LightningContainer {
    
    private final XboxController controller = new XboxController(JoystickConstants.DRIVER_CONTROLLER);

    private final Drivetrain drivetrain = new Drivetrain();

    @Override
    protected void configureAutonomousCommands() {}

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
