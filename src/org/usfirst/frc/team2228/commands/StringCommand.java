package org.usfirst.frc.team2228.commands;

import org.usfirst.frc.team2228.robot.DebugLogger;
import edu.wpi.first.wpilibj.command.Command;

public class StringCommand extends Command {
    private String myLine;
    private boolean isDone;
	
	public StringCommand(String out) {
		myLine = out;
		isDone = false;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("String initialized");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println(myLine);
		DebugLogger.log(myLine);
		isDone = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isDone;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("String end");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
