/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5242.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	private Spark leftSpark = new Spark(0);  // pwm port 0 - left motor controller
	private Spark rightSpark = new Spark(1); // pwm port 1 - right motor controller
	
	private DifferentialDrive drive = new DifferentialDrive(leftSpark, rightSpark); //2 motor drive
	private Joystick joystick = new Joystick(0);
	
	private Solenoid solXpand = new Solenoid(0);
	private Solenoid solRtract = new Solenoid(1);
	
	private Timer m_timer = new Timer();
	private double driveSpeed = -0.65;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (m_timer.get() < 1.0) {
			drive.tankDrive(0.5, 0.5); // drive forwards half speed
		} else {
			drive.stopMotor(); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		drive.tankDrive(joystick.getRawAxis(1)*driveSpeed,joystick.getRawAxis(5)*driveSpeed);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	}
}
