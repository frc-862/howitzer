package com.lightningrobotics.howitzer.commands;

import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final XboxController controller;

    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public SwerveDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.controller = controller;
    }

    @Override
    public void execute() {
        // final var xSpeed = -xspeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft)) * DrivetrainConstants.MAX_SPEED;
        // final var ySpeed = -yspeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft)) * DrivetrainConstants.MAX_SPEED;
        // final var rot = -rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight)) * DrivetrainConstants.MAX_ANGULAR_SPEED;

        final var xSpeed = -controller.getX(GenericHID.Hand.kLeft) * DrivetrainConstants.MAX_SPEED;
        final var ySpeed = -controller.getY(GenericHID.Hand.kLeft) * DrivetrainConstants.MAX_SPEED;
        final var rot = -controller.getX(GenericHID.Hand.kRight) * DrivetrainConstants.MAX_ANGULAR_SPEED;

        boolean fieldRelative = !controller.getBumper(GenericHID.Hand.kRight);

        drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

}
