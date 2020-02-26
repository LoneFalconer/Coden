/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

  XboxController controller;
  Joystick driver;

  public OI() {
    driver = new Joystick(0);
    controller = new XboxController(1);
  }
  public Joystick getDriverJoystick(){
    return driver;
  }
  public XboxController getController(){
    return controller;
  }
}
