/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import org.opencv.core.TickMeter;

//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.command.Command;  
//import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // public static Drivetrain drivetrainSub = new Drivetrain();
  // public static BallLower ballLowerSub =  new BallLower();
  // public static BallShoot ballShootSub = new BallShoot();
  // public static Lift liftSub = new Lift();
  public static OI oi;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  // CANSparkMax leftShooter;
  // CANSparkMax rightShooter;

  // CANSparkMax elevator;
  // CANSparkMax arm;

   XboxController controller;

   CANSparkMax frontRight;
   CANSparkMax frontLeft;
   CANSparkMax rearRight;
   CANSparkMax rearLeft;
   CANSparkMax winch;
   CANSparkMax arm;
   CANSparkMax intake;

   SpeedControllerGroup left;
   SpeedControllerGroup right;

   DoubleSolenoid solenoidLowLevel = new DoubleSolenoid(0, 1);
   DoubleSolenoid solenoidHighLevel = new DoubleSolenoid(7, 6);
   DoubleSolenoid solenoidDoor = new DoubleSolenoid(2,4);

  Compressor comp = new Compressor(0);

  private final Timer m_timer = new Timer();
  private boolean m_areWeThereYet = false;

  @Override
  public void robotInit() {
    oi = new OI();
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(256, 144);
    camera.setFPS(30);


    frontRight = new CANSparkMax(RobotMap.Right_Front_Drive_Motor, MotorType.kBrushless);
    frontLeft = new CANSparkMax(RobotMap.Left_Front_Drive_Motor, MotorType.kBrushless);
    rearRight = new CANSparkMax(RobotMap.Right_Rear_Drive_Motor, MotorType.kBrushless);
    rearLeft = new CANSparkMax(RobotMap.Left_Rear_Drive_Motor, MotorType.kBrushless);

    winch = new CANSparkMax(RobotMap.Climb_Winch, MotorType.kBrushless);
    arm = new CANSparkMax(RobotMap.Climb_Arm, MotorType.kBrushless);
    intake = new CANSparkMax(RobotMap.Ball_Intake, MotorType.kBrushed);
   
    left = new SpeedControllerGroup(frontLeft, rearLeft);
    right = new SpeedControllerGroup(frontRight, rearRight);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_timer.start();
    m_areWeThereYet = false;
    left.set(1);
    right.set(1);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    if(m_areWeThereYet == true){
      return;
    }

    if(m_timer.hasPeriodPassed(3)){
      left.set(0);
      right.set(0);
      m_areWeThereYet = true;
    }


  }

  @Override
  public void teleopInit() {
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    if (oi.getController().getAButtonPressed()){
      solenoidLowLevel.set(Value.kReverse);
    }
    if (oi.getController().getYButtonPressed()){
      solenoidHighLevel.set(Value.kForward);
    }
    if (oi.getController().getXButtonPressed()){
      solenoidLowLevel.set(Value.kForward);
    }
    if (oi.getController().getBButtonPressed()){
      solenoidHighLevel.set(Value.kReverse);
    }
    if (oi.getController().getBumperPressed(Hand.kLeft)){
      solenoidDoor.set(Value.kReverse);
    }
    if (oi.getController().getBumperPressed(Hand.kRight)){
      solenoidDoor.set(Value.kForward);
    }
    double triggerR = oi.getController().getTriggerAxis(Hand.kRight);
    double triggerL = oi.getController().getTriggerAxis(Hand.kLeft);
    if (oi.getController().getStartButtonPressed()){
      winch.set(-.4);
    }
    if (oi.getController().getStartButtonReleased()){
      winch.set(0);
    }
    if (oi.getController().getBackButtonReleased()){
      winch.set(0);
    }
    double xleft = oi.getController().getX(Hand.kLeft);
    double yleft = oi.getController().getY(Hand.kLeft);
    double xright = oi.getController().getX(Hand.kRight);
    double yright = oi.getController().getY(Hand.kRight);
    if (triggerL > .5){
      intake.set(.4);
    }
    if (triggerR > .5){
      intake.set(-.4);
    }
    if (triggerL < .5 && triggerR < .5){
      intake.set(0);
    }
    if (Math.abs(xleft) > 0.1 || Math.abs(yleft) > 0.1){

    }
    if (Math.abs(xright) > 0.1 || Math.abs(yright) > 0.1){

    }

    int pov = oi.getController().getPOV();

    if(pov != -1){
      if(pov == 180){
        arm.set(.1);
      }
      if(pov == 0){
        arm.set(-.1);
      }
    }
    if(pov == -1){
      arm.set(0);
    }

    double xinput = -oi.getDriverJoystick().getX();
    double yinput = -oi.getDriverJoystick().getY();
    //new code added lines 209-214

    if (Math.abs(xinput) < 0.2 && Math.abs(yinput) < 0.2){
      left.set(0);
      right.set(0); 
      return; 
    }
    if (oi.getDriverJoystick().getTriggerPressed()){
      System.out.println("joytrig");
    }
    
    left.set(yinput-xinput);
    right.set(-yinput-xinput);

  }



  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testInit(){
    m_timer.start();
  }
  @Override
  public void testPeriodic() {
    if(!m_timer.hasPeriodPassed(3))
      return;

    boolean e = comp.enabled();
    boolean clc = comp.getClosedLoopControl();
    double cc = comp.getCompressorCurrent();
    boolean ccthf = comp.getCompressorCurrentTooHighFault();
    boolean ccthsf = comp.getCompressorCurrentTooHighStickyFault();
    boolean cncf = comp.getCompressorNotConnectedFault();
    boolean cncsf = comp.getCompressorNotConnectedStickyFault();
    boolean csf = comp.getCompressorShortedFault();
    boolean cssf = comp.getCompressorShortedStickyFault();
    boolean psv = comp.getPressureSwitchValue();

    System.out.println(e + " comp enabled\n" +
    clc + " ClosedLoopControl\n" +
    cc + " CompressorCurrent\n" +
    ccthf + " CompressorCurrentTooHighFault\n" +
    ccthsf + " CompressorCurrentTooHighStickyFault\n" +
    cncf + " CompressorNotConnectedFault\n" +
    cncsf + " CompressorNotConnectedStickyFault\n" +
    csf + " CompressorShortedFault\n" +
    cssf + " CompressorShortedStickyFault\n" +
    psv + " PressureSwitchValue");

    if (oi.getController().getAButtonPressed()){
      System.out.println("A Button pressed - reverse");
    }
    if (oi.getController().getYButtonPressed()){
      System.out.println("Y Button pressed - forward");
    }
    if (oi.getController().getXButtonPressed()){
      System.out.println("X Button pressed");
    }
    if (oi.getController().getBButtonPressed()){
      System.out.println("B button pressed");
    }
    if (oi.getController().getBumperPressed(Hand.kLeft)){
      System.out.println("left bumper");
    }
    if (oi.getController().getBumperPressed(Hand.kRight)){
      System.out.println("right bumper");
    }
    double triggerR = oi.getController().getTriggerAxis(Hand.kRight);
    double triggerL = oi.getController().getTriggerAxis(Hand.kLeft);

      System.out.println("trigger left"+(triggerL));

      System.out.println("trigger right"+(triggerR));

    if (oi.getController().getStartButtonPressed()){
      System.out.println("Start button pushed");
    }
    if (oi.getController().getStartButtonReleased()){
      System.out.println("Start button released");
    }
    if (oi.getController().getBackButtonPressed()){
      System.out.println("Back button pushed");
    }
    if (oi.getController().getBackButtonReleased()){
      System.out.println("Back button pushed");
    }

    //DoubleSolenoid.Value dsv = solenoidHighLevel.get();
    //System.out.println(dsv + " solenoid High Level get Results");

    //this line in the () gives you units in 45s it is an int (writes the same as the doulbe up there)
    System.out.println("Get POV Count"+(oi.getController().getPOVCount()));
    System.out.println("Get POV"+(oi.getController().getPOV()));

    double xleft = oi.getController().getX(Hand.kLeft);
    double yleft = oi.getController().getY(Hand.kLeft);
    double xright = oi.getController().getX(Hand.kRight);
    double yright = oi.getController().getY(Hand.kRight);

      System.out.println("xbox left joystick"+(xleft)+" "+(yleft));
  
      System.out.println("xbox right joystick"+(xright)+" "+(yright));

      double xinput = -oi.getDriverJoystick().getX();
      double yinput = -oi.getDriverJoystick().getY();

      System.out.println("left and right "+(yinput-xinput)+" "+(-yinput-xinput));

      System.out.println("frontRight falts =" + frontRight.getFaults() + " last eror =" + frontRight.getLastError());
      System.out.println("frontLeft falts =" + frontLeft.getFaults() + " last eror =" + frontLeft.getLastError());
      System.out.println("rearRight falts =" + rearRight.getFaults() + " last eror =" + rearRight.getLastError());
      System.out.println("rearLeft falts =" + rearLeft.getFaults() + " last eror =" + rearLeft.getLastError());
      System.out.println("wimch falts =" + winch.getFaults() + " last eror =" + winch.getLastError());
      System.out.println("arm falts =" + arm.getFaults() + " last eror =" + arm.getLastError());
      System.out.println("intake falts =" + intake.getFaults() + " last eror =" + intake.getLastError());
  }
}
