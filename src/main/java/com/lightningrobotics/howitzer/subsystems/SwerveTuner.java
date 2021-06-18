package com.lightningrobotics.howitzer.subsystems;

import com.lightningrobotics.howitzer.subsystems.Drivetrain.Modules;
import com.lightningrobotics.howitzer.util.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveTuner extends SubsystemBase {

    private final Drivetrain drivetrain;

    private SwerveModuleState targetTuningState = new SwerveModuleState(1d, Rotation2d.fromDegrees(0d));

    private NetworkTableEntry speedEntry;
    private NetworkTableEntry angleEntry;

    public SwerveTuner(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        var tab = Shuffleboard.getTab("ModuleDrive_PID_Tuner");
        tab.addString("Target", targetTuningState::toString);
        tab.addString("Real FL", () -> drivetrain.modules[Modules.FRONT_LEFT.getIdx()].getState().toString());
        tab.addString("Real FR", () -> drivetrain.modules[Modules.FRONT_RIGHT.getIdx()].getState().toString());
        tab.addString("Real BL", () -> drivetrain.modules[Modules.BACK_LEFT.getIdx()].getState().toString());
        tab.addString("Real BR", () -> drivetrain.modules[Modules.BACK_RIGHT.getIdx()].getState().toString());
        speedEntry = tab.add("Speed", 0d).getEntry();
        angleEntry = tab.add("Angle", 0d).getEntry();
    }

    @Override
    public void periodic() {

        if(DriverStation.getInstance().isEnabled()) {

            targetTuningState = new SwerveModuleState(speedEntry.getDouble(0d), Rotation2d.fromDegrees(angleEntry.getDouble(0d)));

            var states = new SwerveModuleState[]{targetTuningState, targetTuningState, targetTuningState, targetTuningState};
            drivetrain.setModuleStates(states);

        } else {
            drivetrain.stop();
        }
        
    
    }
    
}
