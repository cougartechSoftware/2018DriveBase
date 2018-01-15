package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.JustDriveCommand;
import org.usfirst.frc.team2228.commands.StringCommand;
import org.usfirst.frc.team2228.commands.WaitCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousManager {
	boolean finishedAuto = true;
	Joystick joystick;
	private SRXDriveBase driveBase;
    String autoSelected;
    SendableChooser autoChooser;
	Command autonomousCommand = null;
    final String defaultAuto = "Default";
    final String customAuto = "Custom Auto";
    final String anotherAuto = "Another Auto";
    final String driveStraightCalibration = "Drive Straight!";
	
	SRXDriveBase drive;
	
	public AutonomousManager(SRXDriveBase _driveBase, Joystick _joystick){
		drive = _driveBase;	
		joystick = _joystick;
		
		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault("Default Auto", defaultAuto);
		autoChooser.addObject("Custom Auto", customAuto);
		autoChooser.addObject("Another Auto", anotherAuto);
		autoChooser.addObject("Drive Straight Calibration", driveStraightCalibration);
	    SmartDashboard.putData("Auto choices", autoChooser);
	}
		
	public void autonomousInit(SRXDriveBase _driveBase) {
		driveBase = _driveBase;
		driveBase.setLeftEncPositionToZero();
		autoSelected = (String) autoChooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		Scheduler scheduler = Scheduler.getInstance();
		
		DebugLogger.log("autoInit");
		
	    switch (autoSelected) {
		case customAuto:
			//scheduler.add(new WaitCommand(0.5));
			autonomousCommand = new WaitCommand(0.5, 
					new JustDriveCommand(drive, 0.25, 0.07));
			DebugLogger.log("start wait command");
			//scheduler.add(new JustDriveCommand(driveBase, 0.25, 0.08));
			break;
		case anotherAuto:
			CommandGroup groupCommand = new CommandGroup();
			groupCommand.addSequential(new WaitCommand(0.5, null));
			groupCommand.addSequential(new JustDriveCommand(drive, 0.25, 0.07));
			autonomousCommand = groupCommand;
			break;
		case defaultAuto:
		default:
			autonomousCommand = new StringCommand("doNothing!");
			break;
		case driveStraightCalibration:
			autonomousCommand = new StringCommand("drive straight!");
			break;
		}
	
	// schedule the autonomous command (example)
	
	if (autonomousCommand != null)
	   autonomousCommand.start();
	}
	
	public void AutoPeriodic(SRXDriveBase _driveBase){
		driveBase = _driveBase;
//		if (finishedAuto) {
//			if (!driveBase.testDriveStraightCalibration(40.0, .4)) {
//				finishedAuto = false;
//				System.out.println("Stop moving forwards!!!!");
//			}
//		}
		
		
//		boolean buttonA = joystick.getRawButton(XBoxConfig.Y_BUTTON);
//		SmartDashboard.getNumber("Right Correction Factor", SRXDriveBaseCfg.kDriveStraightCorrection);
//		if (buttonA /*&& lastButtonRead*/) {
//			finishedAuto = true;
//			System.out.println("Button1 is pressed");
//
//		} else if (finishedAuto) {
//			System.out.println("Button2 is pressed");
//			if (!driveBase.testDriveStraightCalibration(40.0, .4)) {
//				finishedAuto = false;
//			}
//		}
	}
	
    public void killAuto(){
    	DebugLogger.log("kill auto");
    	if (autonomousCommand != null)
    		   autonomousCommand.cancel();
    }

}
