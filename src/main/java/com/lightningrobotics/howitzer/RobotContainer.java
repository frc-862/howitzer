package com.lightningrobotics.howitzer;

import com.lightningrobotics.common.LightningContainer;
import com.lightningrobotics.common.command.drivetrain.swerve.SwerveDriveCommand;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer extends LightningContainer {

    private static final XboxController driver = new XboxController(0);

    private static final LightningIMU imu = LightningIMU.navX();

    private static final SwerveDrivetrain drivetrain = new Drivetrain();

    @Override
    protected void configureButtonBindings() { }

    @Override
    protected void configureSystemTests() { }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, imu, driver, true));
    }

    @Override
    protected void releaseDefaultCommands() { }

    @Override
    protected void initializeDashboardCommands() {
    }

    @Override
    protected void configureAutonomousPaths() { }

    @Override
    protected void configureAutonomousCommands() { }

    @Override
    protected void configureFaultCodes() { }

    @Override
    protected void configureFaultMonitors() { }

    @Override
    public LightningDrivetrain getDrivetrain() {
        return drivetrain;
    }

}
