package com.lightningrobotics.howitzer.commands;

import com.lightningrobotics.howitzer.subsystems.Drivetrain;
import com.lightningrobotics.howitzer.subsystems.SwerveModule;
import com.lightningrobotics.howitzer.subsystems.Drivetrain.Modules;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lightning.util.JoystickFilter;

public class ModuleTest extends CommandBase {

    private final SwerveModule module;
    private final XboxController controller;
    private final JoystickFilter filter;

    public ModuleTest(Drivetrain drivetrain, Modules module, XboxController controller) {
        this.module = drivetrain.modules[module.getIdx()];
        addRequirements(drivetrain); // we do not add IMU as a requirement because it's use is read-only
        this.controller = controller;
        this.filter = new JoystickFilter(0.1d, 0.1d, 1d, JoystickFilter.Mode.CUBED);
    }

    @Override
    public void execute() {
        module.setRawAzimuthPower(filter.filter(-controller.getX(GenericHID.Hand.kLeft)));
        module.setRawDrivePower(filter.filter(-controller.getX( GenericHID.Hand.kRight)));
        //module.setRawDrivePower(1d);
    }

    @Override
    public void end(boolean interrupted) {
        module.setRawAzimuthPower(0d);
        module.setRawDrivePower(0d);
    }

}
