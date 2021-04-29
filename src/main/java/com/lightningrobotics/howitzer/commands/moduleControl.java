package com.lightningrobotics.howitzer.commands;

import com.lightningrobotics.howitzer.subsystems.Drivetrain;
import com.lightningrobotics.howitzer.subsystems.SwerveModule;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class moduleControl extends CommandBase {

    private SwerveModule module;
    private SwerveModuleState state;
    private Drivetrain drivetrain;
    private ModuleNumber moduleNumber;
    private double speed;
    private Rotation2d angle;

    public enum ModuleNumber {
        FRONT_LEFT_MOTOR(0),
        FRONT_RIGHT_MOTOR(1),
        BACK_LEFT_MOTOR(2),
        BACK_RIGHT_MOTOR(3);
        private int value;
        private ModuleNumber(int motorNumber){
            value = motorNumber;
        }
        public int getValue() {
            return value;
        }
    }

    public moduleControl(Drivetrain drivetrain, ModuleNumber moduleNumber, double speed, Rotation2d angle) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.angle = angle;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        module = drivetrain.modules[moduleNumber.getValue()];
        state = new SwerveModuleState(speed, angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        module.setDesiredState(state);
    }

    @Override
    public void end(boolean interrupted) {
        state = new SwerveModuleState(0, module.getAngle());
        module.setDesiredState(state);
    }
}
