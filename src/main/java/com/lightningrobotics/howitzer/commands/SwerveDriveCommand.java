package com.lightningrobotics.howitzer.commands;

import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final XboxController controller;

    public SwerveDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.controller = controller;
    }

    @Override
    public void execute() {
        //TODO fix these variable names
        final double xSpeed = -controller.getY(GenericHID.Hand.kLeft) * DrivetrainConstants.MAX_SPEED; // The y axis of the left joysick corresponds to the yspeed (variable name is wrong)
        final double ySpeed = -controller.getX(GenericHID.Hand.kLeft) * DrivetrainConstants.MAX_SPEED;  // The x axis of the left joysick corresponds to the xspeed (variable name is wrong)
        final double rot = -controller.getX(GenericHID.Hand.kRight) * DrivetrainConstants.MAX_ANGULAR_SPEED; // The x axis of the right joystick corresponds to the rotation (yaw) of the robot

        drivetrain.drive(xSpeed, ySpeed, rot);
    }

}
