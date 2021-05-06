// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.howitzer.commands.auto;

import java.util.Arrays;
import java.util.List;

import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lightning.subsystems.LightningDrivetrain;

public class TestPathCommand extends CommandBase {

	private static final Pose2d ZERO_POSE2D = new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d));

  	public SwerveControllerCommand getCommand(Drivetrain drivetrain, List<Pose2d> coordinates) {
		TrajectoryConfig config = new TrajectoryConfig(drivetrain.getMaxVelocity(), drivetrain.getMaxAcceleration());
	
		config.setKinematics(drivetrain.getKinematics());
		config = config.setReversed(false);
	
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(coordinates, config);
	
		// Translate to robot-relative trajectory
		Transform2d transform = ZERO_POSE2D.minus(trajectory.getInitialPose());
		Trajectory translatedTrajectory = trajectory.transformBy(transform);
	
		SwerveControllerCommand cmd = new SwerveControllerCommand(
		    translatedTrajectory,
		    drivetrain::getOdometryPose2d,
		    drivetrain.getKinematics(),
		    drivetrain.getXPidController(),
		    drivetrain.getYPidController(),
		    drivetrain.getThetaPidController(),
			drivetrain::getOdometryRotation2d,
			drivetrain::setModuleStates,
		    drivetrain
		);
	
		return cmd;

	}

	
  /** Creates a new TestPathCommand. */
  public TestPathCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
