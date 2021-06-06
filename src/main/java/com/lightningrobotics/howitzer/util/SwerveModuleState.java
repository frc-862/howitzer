package com.lightningrobotics.howitzer.util;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveModuleState {

    public double velocity;
    public Rotation2d angle;

    public SwerveModuleState(double velocity, Rotation2d angle) {
        this.velocity = velocity;
        this.angle = angle;
    }

    public static SwerveModuleState optimize(SwerveModuleState target, Rotation2d currentRotation) {
        var delta = target.angle.minus(currentRotation);
        if(Math.abs(delta.getDegrees()) > 90d) {
            return new SwerveModuleState(-target.velocity, target.angle.rotateBy(Rotation2d.fromDegrees(180d)));
        }
        return target;
    }

}
