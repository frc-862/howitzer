package com.lightningrobotics.howitzer;

import com.lightningrobotics.howitzer.Constants.JoystickConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;
import com.lightningrobotics.howitzer.commands.SwerveDriveCommand;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;
import com.lightningrobotics.howitzer.util.LightningIMU;

import edu.wpi.first.wpilibj.XboxController;
import frc.lightning.LightningConfig;
import frc.lightning.LightningContainer;
import frc.lightning.subsystems.LightningDrivetrain;

public class RobotContainer extends LightningContainer {

    // GAMEPADS
    private static final XboxController driver = new XboxController(JoystickConstants.DRIVER_CONTROLLER);

    // ROBOT COMPONENTS
    private static final LightningIMU imu = LightningIMU.pigeon(RobotMap.PIGEON);
	
    // SUBSYSTEMS
    private static final Drivetrain drivetrain = new Drivetrain();
	
    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, imu, driver, true));
    }

    @Override
    protected void initializeDashboardCommands() { }

    @Override
    protected void configureAutonomousCommands() { }
    
    @Override
    protected void configureButtonBindings() { }

    @Override
    protected void configureSystemTests() { }

    @Override
    public LightningConfig getConfig() {
        return null;
    }

    @Override
    public LightningDrivetrain getDrivetrain() {
        return null;
    }

    @Override
    protected void releaseDefaultCommands() { }

}

