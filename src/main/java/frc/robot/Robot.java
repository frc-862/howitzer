/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lightning.LightningContainer;
import frc.lightning.LightningRobot;
import frc.robot.containers.*;

import java.nio.file.Files;
import java.nio.file.Paths;

public class Robot extends LightningRobot {

    public static final boolean DRIVETRAIN_LOGGING_ENABLED = true;

    public Robot() {
        super(getRobot());
    }

    private static LightningContainer getRobot() {
        if(isHowitzer()) return new HowitzerContainer();
        return null;
    }

    public static boolean isHowitzer() {
        return Files.exists(Paths.get("/home/lvuser/howitzer"));
    }

}
