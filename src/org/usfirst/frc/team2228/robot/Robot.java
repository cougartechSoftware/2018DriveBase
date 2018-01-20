package org.usfirst.frc.team2228.robot;

import java.io.File;

import org.usfirst.frc.team2228.commands.JustDriveCommand;
import org.usfirst.frc.team2228.commands.StringCommand;
import org.usfirst.frc.team2228.commands.WaitCommand;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String branchStr = "2018DriveBase";
	final String versionStr = " 0.2.3";
	//dumb comment
	private Joystick joystick;
	private SRXDriveBase driveBase;
	private TeleopController chessyDrive;
	private AutonomousManager autoMgr;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		SmartDashboard.putString("version #", (branchStr + versionStr));
		
		// put axis and button mapping in DriverConfig
		joystick = new Joystick(RobotMap.JOYSTICK_1);
		driveBase = new SRXDriveBase();
		chessyDrive = new TeleopController(joystick, driveBase);
		autoMgr = new AutonomousManager(driveBase, joystick);
		
		  
		File _logDirectory = new File("/home/lvuser/log");
		if (!_logDirectory.exists()) {
			_logDirectory.mkdir();
		}
		DebugLogger.init("/home/lvuser/log/Debug_");
		DebugLogger.log("robotInit" + branchStr + versionStr);
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoMgr.autonomousInit(driveBase);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
//		Scheduler.getInstance().run();
		autoMgr.AutoPeriodic(driveBase);
	}
	
	public void teleopInit() {
		autoMgr.killAuto();
		System.out.println("teleopInit() fi!");
		driveBase.setClearActionFlags();
		chessyDrive.teleopInit();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		chessyDrive.teleopPeriodic();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
    	LiveWindow.run();
	}
}

