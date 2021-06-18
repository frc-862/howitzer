// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.howitzer.subsystems;

import com.lightningrobotics.howitzer.controller.PIDFController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDFDashboardTuner extends SubsystemBase {

	private PIDFController controller;

	private NetworkTableEntry kPTuner;
	private NetworkTableEntry kITuner;
	private NetworkTableEntry kDTuner;
	private NetworkTableEntry kFTuner;

	public PIDFDashboardTuner(String name, PIDFController controller) {
		this.controller = controller;

		var tab = Shuffleboard.getTab(name + "_PID_Tuner");

		kPTuner = tab.add("kP", 0d).getEntry();
		kITuner = tab.add("kI", 0d).getEntry();
		kDTuner = tab.add("kD", 0d).getEntry();
		kFTuner = tab.add("kF", 0d).getEntry();

		kPTuner.setDouble(controller.getkP());
		kITuner.setDouble(controller.getkI());
		kDTuner.setDouble(controller.getkD());
		kFTuner.setDouble(controller.getkF());

	}

	@Override
	public void periodic() {

		controller.setkP(kPTuner.getDouble(controller.getkP()));
		controller.setkI(kITuner.getDouble(controller.getkI()));
		controller.setkD(kDTuner.getDouble(controller.getkD()));
		controller.setkF(kFTuner.getDouble(controller.getkF()));

	}

}
