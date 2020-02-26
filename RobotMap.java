/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Motor IDs
  public static int
  Left_Front_Drive_Motor = 1,
  Left_Rear_Drive_Motor = 4,
  Right_Front_Drive_Motor = 3,
  Right_Rear_Drive_Motor = 2,

  Ball_Intake = 5,

  Climb_Arm = 6,
  Climb_Winch = 7;

  //Ball_Left = 9,
 // Ball_Right = 10,

 // Tilt_Motor = 11,
  //Lift_Motor = 12,

  //Tilt and Lift pos values
  //Tilt_pos = 2,
  //Lift_pos = 0,

  //Limit Switch DIO
  //Tilt_LS = 0,
  //Lift_LS = 1;

  //Encoder DIO Values
  //Tilt_Channel_A = 0,
  //Tilt_Channel_B = 1,

  //Lift_Channel_A = 2,
  //Lift_Channel_B = 3;

  //Encoder Threshold Values
  //public static double
  //Tilt_Upper_Thresh = 0,
  //Tilt_Middle_Thresh = -18000,
  //Tilt_Lower_Thresh = -23400,
  
  //Lift_Upper_Thresh = 0,
  //Lift_Middle_Thresh = 0,
  //Lift_Lower_Thresh = 0;
}
