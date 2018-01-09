package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.DebugLogger;
import org.usfirst.frc.team2228.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class WaitCommand extends Command {
	private boolean isDone = false;
	private double startTime = 0;
	private double duration = 0;
	private Command nextCommand = null;
	
	public WaitCommand(double waitTime,
			           Command next) {
		isDone = false;
		duration = waitTime;
		nextCommand = next;
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("wait initialized");
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Timer.getFPGATimestamp() >= startTime + duration) {		
		   isDone = true;
		   System.out.println("wait done");
		   DebugLogger.log("wait done");
		}
		else {

			   DebugLogger.log("still waiting....");
		}
			
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isDone;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("wait end");
		if (nextCommand != null){
			nextCommand.start();
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}

