package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.DebugLogger;
import org.usfirst.frc.team2228.robot.Robot;
import org.usfirst.frc.team2228.robot.SRXDriveBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class JustDriveCommand extends Command {
	private boolean isDone = false;
	SRXDriveBase drive;
	double speed;
	
	// timeout in seconds
	public JustDriveCommand(SRXDriveBase driveBase,
			                    double       driveSpeed,
			                    double       timeOut) {
		super(timeOut);
		drive = driveBase;
		speed = driveSpeed;
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		DebugLogger.log("JustDrive starting"  + speed + " percent");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		drive.WPISetThrottleTurn(speed, 0);
		DebugLogger.log("JustDrive sending speed"  + speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		drive.stopMotors();
		System.out.println("just drive done");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}