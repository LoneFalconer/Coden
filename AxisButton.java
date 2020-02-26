/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class AxisButton extends Button{
    Joystick joy;
    int axis;
    double threshold;
    public AxisButton(Joystick joystick, int i_axis, double i_threshold) {
        joy = joystick;
        axis = i_axis;
        threshold = i_threshold;
    }

    public boolean get(){
         return joy.getRawAxis(axis) > threshold;
    }
}
