// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

	private static final XboxController controller = new XboxController(JoystickConstants.DRIVER_CONTROLLER);

	private static final LightningIMU imu = LightningIMU.pigeon(RobotMap.PIGEON);
	
	private static final Drivetrain drivetrain = new Drivetrain(imu);

	@Override
	protected void configureDefaultCommands() {
		drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, imu, controller, true));
	}

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
	protected void initializeDashboardCommands() { }

	@Override
	protected void releaseDefaultCommands() { }

}
