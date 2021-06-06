package com.lightningrobotics.howitzer.util;

import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveKinematics {

    private static class Wheelbase {
        public final double W; // Width
        public final double L; // Length
        public final double R; // Diagonal
        Wheelbase(double width, double length) {
            this.W = width;
            this.L = length;
            this.R = Math.sqrt((W * W) + (L * L));
        }
    }

    private final Wheelbase wb;

    private SwerveModuleState[] states;

    public SwerveKinematics(double width, double length) {
        this.wb = new Wheelbase(width, length);
        this.states = new SwerveModuleState[DrivetrainConstants.NUM_MODULES];
    }
    
    public SwerveModuleState[] inverse(DrivetrainSpeed speed) {

        var a = speed.vy - speed.omega * (wb.L / wb.R);
        var b = speed.vy + speed.omega * (wb.L / wb.R);
        var c = speed.vx - speed.omega * (wb.W / wb.R);
        var d = speed.vx + speed.omega * (wb.W / wb.R);

        states[Drivetrain.Modules.FRONT_RIGHT.getIdx()]  = new SwerveModuleState(Math.sqrt((b*b)+(c*c)), new Rotation2d(Math.atan2(b,c)));
        states[Drivetrain.Modules.FRONT_LEFT.getIdx()]   = new SwerveModuleState(Math.sqrt((b*b)+(d*d)), new Rotation2d(Math.atan2(b,d)));
        states[Drivetrain.Modules.BACK_RIGHT.getIdx()]   = new SwerveModuleState(Math.sqrt((a*a)+(c*c)), new Rotation2d(Math.atan2(a,c)));
        states[Drivetrain.Modules.BACK_LEFT.getIdx()]    = new SwerveModuleState(Math.sqrt((a*a)+(d*d)), new Rotation2d(Math.atan2(a,d)));

        // Normalize if necessary
        var max = UtilMath.max(states[Drivetrain.Modules.FRONT_RIGHT.getIdx()].velocity,
                                states[Drivetrain.Modules.FRONT_LEFT.getIdx()].velocity,
                                states[Drivetrain.Modules.BACK_RIGHT.getIdx()].velocity,
                                states[Drivetrain.Modules.BACK_LEFT.getIdx()].velocity);
        
        if(max > DrivetrainConstants.MAX_SPEED) {
            states[Drivetrain.Modules.FRONT_RIGHT.getIdx()].velocity /= max;
            states[Drivetrain.Modules.FRONT_LEFT.getIdx()].velocity /= max;
            states[Drivetrain.Modules.BACK_RIGHT.getIdx()].velocity /= max;
            states[Drivetrain.Modules.BACK_LEFT.getIdx()].velocity /= max;
        }

        return states;

    }

    public SwerveModuleState[] getStates() {
        return states;
    }

}
