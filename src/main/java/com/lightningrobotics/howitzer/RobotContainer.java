package com.lightningrobotics.howitzer;

import com.lightningrobotics.howitzer.Constants.JoystickConstants;
import com.lightningrobotics.howitzer.commands.SwerveDriveCommand;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
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
    protected void initializeDashboardCommands() {}

    @Override
    protected void releaseDefaultCommands() {}
    
}
