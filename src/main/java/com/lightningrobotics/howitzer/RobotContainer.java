// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.howitzer;

import com.lightningrobotics.howitzer.Constants.JoystickConstants;
import com.lightningrobotics.howitzer.Constants.ModuleConstants;
import com.lightningrobotics.howitzer.Constants.RobotMap;
import com.lightningrobotics.howitzer.commands.ModuleTest;
import com.lightningrobotics.howitzer.commands.SwerveDriveCommand;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;
import com.lightningrobotics.howitzer.subsystems.PIDFDashboardTuner;
import com.lightningrobotics.howitzer.subsystems.SwerveTuner;
import com.lightningrobotics.howitzer.subsystems.Drivetrain.Modules;
import com.lightningrobotics.howitzer.util.LightningIMU;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lightning.LightningConfig;
import frc.lightning.LightningContainer;
import frc.lightning.subsystems.LightningDrivetrain;

public class RobotContainer extends LightningContainer {

	// CONSTANTS
	private static final boolean TUNING_MODE = false;

	// GAMEPADS
	private static final XboxController driver = new XboxController(JoystickConstants.DRIVER_CONTROLLER);

	// ROBOT COMPONENTS
	private static final LightningIMU imu = LightningIMU.pigeon(RobotMap.PIGEON);
	
	// SUBSYSTEMS
	private static final Drivetrain drivetrain = new Drivetrain(imu);
	
	// TUNERS
	// private static final PIDFDashboardTuner moduleAzimuthTuner;
	// private static final PIDFDashboardTuner moduleDriveTuner;
	// private static final SwerveTuner swerveTuner;

	// TUNER INITIALIZATION
	// static {
	// 	if(TUNING_MODE) {
	// 		moduleAzimuthTuner = new PIDFDashboardTuner("ModuleAzimuth", ModuleConstants.AZIMUTH_CONTROLLER);
	// 		moduleDriveTuner = new PIDFDashboardTuner("ModuleDrive", ModuleConstants.DRIVE_CONTROLLER);
	// 		swerveTuner = new SwerveTuner(drivetrain);
	// 	} else {
	// 		moduleAzimuthTuner = null;
	// 		moduleDriveTuner = null;
	// 		swerveTuner = null;
	// 	}
	// }

	@Override
	protected void configureDefaultCommands() {
		drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, imu, driver, true));
	}

	@Override
	protected void initializeDashboardCommands() {
		// var tab = Shuffleboard.getTab("Module Tests");
		// tab.add("Front Left Module Test", new ModuleTest(drivetrain, Modules.FRONT_LEFT, driver));
		// tab.add("Front Right Module Test", new ModuleTest(drivetrain, Modules.FRONT_RIGHT, driver));
		// tab.add("Back Left Module Test", new ModuleTest(drivetrain, Modules.BACK_LEFT, driver));
		// tab.add("Back Right Module Test", new ModuleTest(drivetrain, Modules.BACK_RIGHT, driver));
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
	protected void releaseDefaultCommands() { }

}
