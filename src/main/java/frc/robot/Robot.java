// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private SwerveDrive swerveDrive = new SwerveDrive();
  //private Joystick logitech3 = new Joystick(0);
  private XboxController xbox1 = new XboxController(0);
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    dashboardSetUp();
    swerveDrive.setInitHeading(Swerve.initHeading);
    swerveDrive.initEncoders();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    swerveDrive.initEncoders();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    //swerveDrive.test(logitech3.getX());
    swerveDrive.drive(xbox1.getLeftX(), xbox1.getLeftY());
    swerveDrive.turnTrigger(xbox1.getLeftTriggerAxis(), xbox1.getRightTriggerAxis(), 1);
    
    dashboardSetUp();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void dashboardSetUp() {
    /*SmartDashboard.putNumber("M0 Encoder", swerveDrive.swerveMods[0].get_angleEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("M1 Encoder", swerveDrive.swerveMods[1].get_angleEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("M2 Encoder", swerveDrive.swerveMods[2].get_angleEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("M3 Encoder", swerveDrive.swerveMods[3].get_angleEncoder().getAbsolutePosition());*/
    
    SmartDashboard.putNumber("Target Angle", swerveDrive.targetHeading);

    /*SmartDashboard.putNumber("M0 Angle", swerveDrive.swerveMods[0].calculatedAngle);
    SmartDashboard.putNumber("M1 Angle", swerveDrive.swerveMods[1].calculatedAngle);
    SmartDashboard.putNumber("M2 Angle", swerveDrive.swerveMods[2].calculatedAngle);
    SmartDashboard.putNumber("M3 Angle", swerveDrive.swerveMods[3].calculatedAngle);
    
    SmartDashboard.putNumber("M0 Error", swerveDrive.swerveMods[0].errorAngle);
    SmartDashboard.putNumber("M1 Error", swerveDrive.swerveMods[1].errorAngle);
    SmartDashboard.putNumber("M2 Error", swerveDrive.swerveMods[2].errorAngle);
    SmartDashboard.putNumber("M3 Error", swerveDrive.swerveMods[3].errorAngle);
    
    SmartDashboard.putBoolean("M0 Correct", swerveDrive.swerveMods[0].correctAngle);
    SmartDashboard.putBoolean("M1 Correct", swerveDrive.swerveMods[1].correctAngle);
    SmartDashboard.putBoolean("M2 Correct", swerveDrive.swerveMods[2].correctAngle);
    SmartDashboard.putBoolean("M3 Correct", swerveDrive.swerveMods[3].correctAngle);
    SmartDashboard.putNumber("M0 volt", swerveDrive.swerveMods[0].inputAngleVoltage);
    SmartDashboard.putNumber("M1 volt", swerveDrive.swerveMods[1].inputAngleVoltage);
    SmartDashboard.putNumber("M2 volt", swerveDrive.swerveMods[2].inputAngleVoltage);
    SmartDashboard.putNumber("M3 volt", swerveDrive.swerveMods[3].inputAngleVoltage);*/

    /*SmartDashboard.putNumber("M0 Drive", swerveDrive.swerveMods[0].get_driveMotor().getEncoder().getVelocity());
    SmartDashboard.putNumber("M1 Drive", swerveDrive.swerveMods[1].get_driveMotor().getEncoder().getVelocity());
    SmartDashboard.putNumber("M2 Drive", swerveDrive.swerveMods[2].get_driveMotor().getEncoder().getVelocity());
    SmartDashboard.putNumber("M3 Drive", swerveDrive.swerveMods[3].get_driveMotor().getEncoder().getVelocity());
    
    SmartDashboard.putNumber("M0 volt", swerveDrive.swerveMods[0].inputDriveVoltage);*/
    
  }
}