package org.usfirst.frc.team2228.robot;

import org.usfirst.frc.team2228.commands.JustDriveCommand;
import org.usfirst.frc.team2228.commands.StringCommand;
import org.usfirst.frc.team2228.commands.WaitCommand;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousManager {
    String autoSelected;
    SendableChooser autoChooser;
	Command autonomousCommand = null;
    final String defaultAuto = "Default";
    final String customAuto = "Custom Auto";
    final String anotherAuto = "Another Auto";
	
	SRXDriveBase drive;
	
	public AutonomousManager(SRXDriveBase _driveBase){
		drive = _driveBase;	
		
		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault("Default Auto", defaultAuto);
		autoChooser.addObject("Custom Auto", customAuto);
		autoChooser.addObject("Another Auto", anotherAuto);
	    SmartDashboard.putData("Auto choices", autoChooser);
	}
		
	public void autonomousInit() {
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
		}
	
	// schedule the autonomous command (example)
	
	if (autonomousCommand != null)
	   autonomousCommand.start();
	}
	
    public void killAuto(){
    	DebugLogger.log("kill auto");
    	if (autonomousCommand != null)
    		   autonomousCommand.cancel();
    }

}
