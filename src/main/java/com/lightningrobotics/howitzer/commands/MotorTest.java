package com.lightningrobotics.howitzer.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MotorTest extends CommandBase {

    TalonFX motor;

    public MotorTest(int ID) {
        motor = new TalonFX(ID);
    }

    @Override
    public void initialize() {        
        motor.set(ControlMode.PercentOutput, 0d);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        motor.set(ControlMode.PercentOutput, 0.1d);
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(ControlMode.PercentOutput, 0d);
    }
    
}
