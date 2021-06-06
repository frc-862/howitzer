package com.lightningrobotics.howitzer.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.lightningrobotics.howitzer.controller.DriveController;
import com.lightningrobotics.howitzer.controller.PIDFController;
import com.lightningrobotics.howitzer.util.SwerveKinematics;
import com.lightningrobotics.howitzer.util.SwerveModuleState;
import com.lightningrobotics.howitzer.util.SwerveOdometry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveTrajectoryCommand extends CommandBase {

	private final Timer timer = new Timer();
	private final Trajectory trajectory;
	private final SwerveOdometry odometer;
	private final SwerveKinematics kinematics;
	private final DriveController controller;
	private final Consumer<SwerveModuleState[]> outputModuleStates;
	private final Supplier<Rotation2d> desiredRotation;

	public SwerveTrajectoryCommand(Trajectory trajectory, SwerveOdometry odometer, SwerveKinematics kinematics,
			PIDFController xController, PIDFController yController, PIDFController thetaController,
			Supplier<Rotation2d> desiredRotation, Consumer<SwerveModuleState[]> outputModuleStates,
			Subsystem... requirements) {

		this.trajectory = trajectory;
		this.odometer = odometer;
		this.kinematics = kinematics;
		this.controller = new DriveController(xController, yController, thetaController);
		this.outputModuleStates = outputModuleStates;
		this.desiredRotation = desiredRotation;
		addRequirements(requirements);

	}

	public SwerveTrajectoryCommand(Trajectory trajectory, SwerveOdometry odometer, SwerveKinematics kinematics,
			PIDFController xController, PIDFController yController, PIDFController thetaController,
			Consumer<SwerveModuleState[]> outputModuleStates, Subsystem... requirements) {
		
		this(trajectory, odometer, kinematics, xController, yController, thetaController,
				() -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
				outputModuleStates, requirements);

	}

	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		double curTime = timer.get();
		var desiredState = trajectory.sample(curTime);

		var targetChassisSpeeds = controller.calculate(odometer.getPose(), desiredState, desiredRotation.get());
		var targetModuleStates = kinematics.inverse(targetChassisSpeeds);

		outputModuleStates.accept(targetModuleStates);
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(trajectory.getTotalTimeSeconds());
	}

}
