package org.usfirst.frc.team2228.robot;
/**
* Class SRXBaseDriveAPI
* RELEASE: 1, RevA 
* Team 2228 / RJV
*
*
*/
/* ===================================
 * REVISIONS:
 * Release 1
 * RevA: original
 */


public class SRXDriveBaseAPI {
/**
*
* =======================================================================================
* SRXBaseDrive SET METHODS
* =======================================================================================


	public void setSRXPercentVbusMode() {
		
	{
	public void setSRXSpeedModeWithFeedback() {
		
	}
	public void setRightEncPositionToZero() {
		
	}
	public void setLeftEncPositionToZero() {

	}
	public void setRightPositionToZero() {
		
	}
	public void setLeftPositionToZero() {
		
	}
	public void setBrakeMode(boolean isBrakeEnabled) {
		
	}
	public void setStopMotors(){
		
	}
	public void setEnableConsoleData(boolean _consoleData){

	}
	public void setEnableLoggingData(boolean _loggingData){
		
	}
*
* =======================================================================================
* STATUS METHODS
* =======================================================================================
*
	public void DisplayChangeParmeters() {
		// displays on smartdashboard values that are changed for tuning
	}
	public void UpdateSRXDriveDataDisplay() {
		// Reads encoder, velocity, current, error, and displays on smartdashboard	
	}
	public void logSRXDriveData(){
		// logs drive data to file
	}
	public double getRightEncoder(){
		// gets the right master raw encoder
	}
	public double getRightEncoderPosition() {
		// gets the right master encoder value with corrected sign for forward
	}
	public double getRightClosedLoopPosition(){
		// gets the closed loop position 
	}
	public double getRightMstrMtrCurrent() {
		// gets the right master motor current
	}
	public double getRightFollowerMtrCurrent() {
		// gets the right follower motor current
	}
	public double getRightClosedLoopVelocity() {
			// gets the right master motor velocity
	}
	public double getRightCloseLoopError() {
		// gets the right master motor close loop error
	}
	public double getLeftEncoder() {
		// gets the raw data from the left encoder
	}
	public double getLeftEncoderPosition() {
		// gets the encoder count with corrected sign for forward
	}
	public double getLeftClosedLoopPosition(){
		// gets the left closed loop position
	}
	public double getLeftFollowerMtrCurrent() {
		// gets left follower motor current
	}
	public double getLeftClosedLoopVelocity() {
		// gets left master motor velocity
	}
	public double getLeftCloseLoopError() {
		// gets left master motor close loop error
	}
	public double getBusVoltage() {
		// gets the bus voltage
	}
	public boolean getIsStallConditionTimedOut() {
		// This method flags a robot in a stall condition longer than stall time
	}
	public boolean getIsMoving {
		// This method returns true is the robot is moving
	}
*
* =======================================================================================
* TELEOP METHODS
* =======================================================================================
*
	
	public void SetDriveTrainCmdLevel(	double _rightCMDLevel, 
										double _leftCMDLevel) {
		
		 // Note: left drive is master drive axis for the robot - the right drive
		 // will be modified for driving straight
		 
		 // NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the
		 // motor bus voltage). Motion command with closed loop reflect speed level
		 // (-1 to 1) * (top motor RPM)
	}

	
	public void WPISetThrottleTurn(	double throttleValue, 
									double turnValue) {
		 // WPI throttle and turn commands This method uses WPI library methods to
		 // drive the robot with a throttle and turn input. Drives were set up by:
		 // driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
		 // throttle would be the game controller Y-axis(joystick fwd/rev) and turn
		 // would be game conctroller X-axis(joystick left/right)
		 
		 // NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
		 // to 1) * (the motor bus voltage). The speed is determined by this power
		 // level and the load to the motor.
	}

	 *
	 * setThrottleTurn is both open loop and closed loop control with drive
	 * straight/drive perpendicular correction
	 *
	public void setThrottleTurn(double _throttleValue, 				// (-1 to 1) rev-neg value/fwd-pos value
								double _turnValue, 					// (-1 to 1) left-neg value/right-pos value
								boolean _isDrivingPerpendicular) {
		// 	_isDrivingPerpendicular -(true): assists drive to have robot move perpendicular into a wall						
	}
	
	public void setFastTurn(boolean _isFastTurnRight) {
		// future TODO
		// If robot blocked by another robot Fast turn turns robot 180 deg to escape
	}

	public double pidDriveCorrection(	boolean _isDrivingPerpend, 
										double _turnValue){
		// This method uses: 
		// 1) The ahrs and a PID equation to provide drive straight correction
		// 2) Two ultrasonic sensors and a PID equation to provide drive perpendicular to a surface correction
	}

	

	*
	* =======================================================================================
	* AUTONOMOUG METHODS
	* =======================================================================================
	*
	public boolean (	double _MoveToPositionIn, 
										double _MoveToPositionPwrLevel, 
										boolean _isCascadeMove) {
		// This method moves the robot with a predetermined power level and stops at
		// the specified position value. The move will be in brake mode to stop
		// method will check that robot is stopped and set brake mode back to coast and respond
		// that move is done
	}
	public boolean movePerpendicularToStop(	double _movePerpendicularPowerLevel, 
											double _movePerpendicularStopIn) {
	}
	public boolean rotateToAngle(	double _rotateToAngle, 
									double _rotatePowerLevel) {
		// direction(true)-rotates right, direction(false)-rotates left
	} 

	public boolean turnByEncoderToAngle(double _turnAngleDeg, 
										double _turnRadiusIn, 
										double _turnPowerLevel, 
										boolean _isDirectionReverse, 
										boolean _isCascadeTurn ) {
	}
	
	
	
	*
	* =======================================================================================
	* SRXDriveBase TEST METHODS
	* =======================================================================================
	*
	public void testMotorSquareWave(boolean _isMtrSquareWaveTestEnabled, 
									boolean _isTestForRightDrive) {
	}

	public boolean testDriveStraightCalibration(double _testDistanceIn, 
												double _pwrLevel){
	} 
	
	
	public boolean delay(double _seconds){
		
	}
	
	*
	* =======================================================================================
	* INDEX AND PROFILE COMMANDS
	* =======================================================================================
	*
	public boolean magicMove(	double _rightCruiseVel, 
								double _rightAccel, 
								double _rightDistance, 
								double _leftCruiseVel,
								double _leftAccel, 
								double _leftDistance) {
		// This method performs a SRX magic motion command from user calculated
		// values
		// User should note that the right drive distance needs to be corrected
		// by kDriveStraightCorrection
	}
	
	
	public boolean SRXBaseDriveIndexRobot(	double indexDistanceIn, 
											int indexTime) {
	
	 *
	 * driveIndexRobot method:
	 * 
	 * @parm indexDistance is in inches
	 * @parm indexTime is in seconds
	 *
	 *       indexRobot uses "Magic Motion" in the Talon SRX modules to index
	 *       the robot. "Magic Motion" SRX method needs cruise velocity(RPM),
	 *       acceleration rate (RPM/Sec), and distance(encoder counts)
	 *
	 *       The following equations for a trapezoid with 1/3 time segments are
	 *       used to determine params for "Magic Motion" move:
	 *
	 *       Velocity = 1.5*(Distance / Time) Accel = Decel = 4.5*(Distance /
	 *       Time2)
	 *
	 *       Velocity(RPM) = (1.5*(Distance(in) / Time(sec))*60(sec/min)) /
	 *       Wheel Circum(in/rev) Acceleration(RPM/sec) = ((1.5*(Distance(in) /
	 *       Time(sec))*60(sec/min)) / Wheel Circum(in/rev)) / Ta(sec) Ta should
	 *       be Time/3 Distance(Encoder Counts) = Distance(in) / (in/count)
	 *
	}
}
*/
}