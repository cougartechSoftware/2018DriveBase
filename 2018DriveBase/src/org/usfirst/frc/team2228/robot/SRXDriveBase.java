package org.usfirst.frc.team2228.robot;
/**
* Class SRXBaseDrive
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

//Carrying over the classes from other libraries
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	private String VersionString = "Release 1, RevB 180105";
	
	// cheesy or tank motors
	private CANTalon driveRightMasterMtr;
	private CANTalon driveRightFollowerMtr;
	private CANTalon driveLeftMasterMtr;
	private CANTalon driveLeftFollowerMtr;
	
	private RobotDrive driveStyle;
	
	private boolean isDriveMoving;
	private double rightDrvTrainTargetPosSetPt;
	private double leftDrvTrainTargetPosSetPt;
	
	//private AHRS IMU;
		
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {
		
		// Create CAN SRX motor controller objects
		driveRightMasterMtr = new CANTalon(RobotMap.CAN_ID_1);
		driveRightFollowerMtr = new CANTalon(RobotMap.CAN_ID_2);
		driveLeftMasterMtr = new CANTalon(RobotMap.CAN_ID_3);
		driveLeftFollowerMtr = new CANTalon(RobotMap.CAN_ID_4);
		
		LiveWindow.addActuator("rtM", "RightMaster", driveRightMasterMtr);
		LiveWindow.addActuator("rtF", "RightFollower", driveRightFollowerMtr);
		LiveWindow.addActuator("lftM", "LeftMaster", driveLeftMasterMtr);
		LiveWindow.addActuator("lftF", "LeftFollower", driveLeftFollowerMtr);
		
		isDriveMoving = false;
		
		/*  Set right/left masters and right/left followers 
		*/
		// Set Right master to speed mode
		driveRightMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveRightMasterMtr.enableControl(); 
		driveRightMasterMtr.set(0);
		
		// Set up right follower
		driveRightFollowerMtr.changeControlMode(TalonControlMode.Follower);
		
		// Set follower motor to follow master
		driveRightFollowerMtr.set(driveRightMasterMtr.getDeviceID());
		driveRightFollowerMtr.enableControl(); 

		// Set left master to speed mode
		driveLeftMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveLeftMasterMtr.enableControl();
		driveLeftMasterMtr.set(0);
		
		// Set up left follower
		driveLeftFollowerMtr.changeControlMode(TalonControlMode.Follower);
		
		// Set follower motor to follow master
		driveLeftFollowerMtr.set(driveLeftMasterMtr.getDeviceID());
		driveLeftFollowerMtr.enableControl(); 
		
		driveRightMasterMtr.setInverted(SRXDriveBaseCfg.isDriveRightMasterMtrReversed);
		driveRightFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveRightFollowerMtrReversed);
		driveLeftMasterMtr.setInverted(SRXDriveBaseCfg.isDriveLeftMasterMtrReversed);
		driveLeftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);

		// Set peak and nominal output voltage levels of motor controllers
		driveRightMasterMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveRightMasterMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		driveRightFollowerMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveRightFollowerMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		driveLeftMasterMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveLeftMasterMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		driveLeftFollowerMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveLeftFollowerMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		
	
		
		/*
		* Set Brake-Coast mode to coast
		*/
		setBrakeMode(SRXDriveBaseCfg.isBrakeEnabled);
		
		/*
		* Setup encoder feedback if there are encoders on master motors
		*/
		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			driveRightMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveRightMasterMtr.reverseSensor(SRXDriveBaseCfg.isRightEncoderSensorReversed);
			driveRightMasterMtr.configEncoderCodesPerRev(SRXDriveBaseCfg.DRIVE_RIGHT_ENCODER_CNTS_PER_REV); 
			
			driveLeftMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveLeftMasterMtr.reverseSensor(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
			driveLeftMasterMtr.configEncoderCodesPerRev(SRXDriveBaseCfg.DRIVE_LEFT_ENCODER_CNTS_PER_REV);
		}
		
		/*
		* Setup PID values if enabled
		*/
		if (SRXDriveBaseCfg.isPIDEnabled) {
			driveRightMasterMtr.setProfile(0);
			driveRightMasterMtr.setF(SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain);
			driveRightMasterMtr.setP(SRXDriveBaseCfg.kdriveRightMstrProportionalGain);
			driveRightMasterMtr.setI(SRXDriveBaseCfg.kdriveRightMstrIntegralGain);
			driveRightMasterMtr.setD(SRXDriveBaseCfg.kdriveRightMstrDerivativeGain);
			
			driveLeftMasterMtr.setProfile(0);
			driveLeftMasterMtr.setF(SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain);
			driveLeftMasterMtr.setP(SRXDriveBaseCfg.kdriveLeftMstrProportionalGain);
			driveLeftMasterMtr.setI(SRXDriveBaseCfg.kdriveLeftMstrIntegralGain);
			driveLeftMasterMtr.setD(SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain);				
		}
		/*
		* Setup closed-loop velocity and Setup PID values if enabled
		*/
		
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			
			driveRightMasterMtr.setPID(SRXDriveBaseCfg.kdriveRightMstrProportionalGain,
										SRXDriveBaseCfg.kdriveRightMstrIntegralGain,
										SRXDriveBaseCfg.kdriveRightMstrDerivativeGain,
										SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain,
										SRXDriveBaseCfg.kdriveRightMstrIzone,
										SRXDriveBaseCfg.kdriveRightMstrRampRate,
										SRXDriveBaseCfg.kdriveRightMstrProfile);
										
			driveLeftMasterMtr.setPID(SRXDriveBaseCfg.kdriveLeftMstrProportionalGain,
										SRXDriveBaseCfg.kdriveLeftMstrIntegralGain,
										SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain,
										SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain,
										SRXDriveBaseCfg.kdriveLeftMstrIzone,
										SRXDriveBaseCfg.kdriveLeftMstrRampRate,
										SRXDriveBaseCfg.kdriveLeftMstrProfile);	
			
			// set controlMode to speed with feedback
			// Note speed is in RPM, thus -1 to 1 input needs to be multiplied by max RPM
			driveRightMasterMtr.changeControlMode(TalonControlMode.Speed);
			driveLeftMasterMtr.changeControlMode(TalonControlMode.Speed);
			
			// ?? to test should we use this for autonomous
			// driveRightMasterMtr.disableNominalClosedLoopVoltage();
			// The following compensates for battery voltage - 50% output would be %50 of 11 volts
			// driveRightMasterMtr.setNominalClosedLoopVoltage(11.0);
			
			// cnts / 4096 cnts/rev(magnetic encoder); 100/4096= 2.4%; 8.7 degrees
			driveRightMasterMtr.setAllowableClosedLoopErr(100);
			driveLeftMasterMtr.setAllowableClosedLoopErr(100);
			
			// Sample period in ms from supported sample periods-default 100ms period/64 sample window
			driveRightMasterMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_25Ms);
			driveLeftMasterMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_25Ms);
			
			// Number if sanples in a rolling average 
			driveRightMasterMtr.SetVelocityMeasurementWindow(16);
			driveLeftMasterMtr.SetVelocityMeasurementWindow(16);
			
			// Activate closed-Loop velocity
			driveRightMasterMtr.set(0);
			driveLeftMasterMtr.set(0);
		}
		
		/*
		* setup IMU if enabled
		*/
		if (SRXDriveBaseCfg.isHeadingModuleEnabled){
			try
			{
			//	imu = new AHRS(SPI.Port.kOnboardCS0);
			}
			catch (RuntimeException ex)
			{
				System.out.println("Error starting the Nav-X");
			}
			//imu.zeroYaw();
		}

		
		driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr);
		
		/*
		* Clear all sticky faults in drive controllers
		*/
		driveRightMasterMtr.clearStickyFaults();	
		driveRightFollowerMtr.clearStickyFaults();
		driveLeftMasterMtr.clearStickyFaults();
		driveLeftFollowerMtr.clearStickyFaults();
		
	
	}	
			
	
	/*================================================================
	* SRXDriveBase commands
	*/
	
	public void setRightPositionToZero() {
		driveRightMasterMtr.setPosition(0);
	}
	
	public void setLeftPositionToZero() {
		driveLeftMasterMtr.setPosition(0);
		
	}
	
	public void setBrakeMode(boolean isBrakeEnabled){
		driveRightMasterMtr.enableBrakeMode(isBrakeEnabled);
		driveRightFollowerMtr.enableBrakeMode(isBrakeEnabled);
		driveLeftMasterMtr.enableBrakeMode(isBrakeEnabled);
		driveLeftFollowerMtr.enableBrakeMode(isBrakeEnabled);
	}
	
	/*=====================================================
	* SRXDriveBase status commands
	*/
	public double getRightPosition() {
		return driveRightMasterMtr.getPosition();
	}
	
	public double getRightMstrMtrCurrent() {
		return driveRightMasterMtr.getOutputCurrent();
	}
	
	public double getRightFollowerMtrCurrent() {
		return driveRightFollowerMtr.getOutputCurrent();
	}
	public double getRightVelocity() {
		return driveRightMasterMtr.getSpeed();		
	}
	
	public double getRightCloseLoopError() {
		return driveRightMasterMtr.getClosedLoopError();
	}
	
	public double getLeftPosition() {
		return driveLeftMasterMtr.getPosition();		
	}
	
	public double getLeftMstrMtrCurrent() {
		return driveLeftMasterMtr.getOutputCurrent();
	}
	public double getLeftFollowerMtrCurrent() {
		return driveLeftFollowerMtr.getOutputCurrent();
	}
	public double getLeftVelocity() {
		return driveLeftMasterMtr.getSpeed();
	}
	
	public double getLeftCloseLoopError() {
		return driveLeftMasterMtr.getClosedLoopError();
	}
	
	/*=========================================================
	* Motion Commands
	*/
	
	 /*
	* Drive power commands. The SRX ESC modules do the rest
	*/
	public void setRightPower(double RightPowerLevel) {
		driveRightMasterMtr.set(RightPowerLevel * SRXDriveBaseCfg.kDriveStraightCorrection) ;
	}
	
	public void setLeftPower(double LeftPowerLevel) {
		driveLeftMasterMtr.set(LeftPowerLevel);
	}
	
	public void arcadeMoveAndRotate(double moveValue, double rotateValue) {
		driveStyle.arcadeDrive(moveValue, rotateValue, false);
	}
	
	
	public void setTurnAndThrottle(double rotateValue, double moveValue){
		double rightMotorSpeed = 0.0;
		double leftMotorSpeed = 0.0;
		
		if (moveValue > 0.0) {  // forward
		   if (rotateValue > 0.0) {
		        leftMotorSpeed = moveValue - rotateValue;
		        rightMotorSpeed = Math.max(moveValue, rotateValue);
		    } else {
		        leftMotorSpeed = Math.max(moveValue, -rotateValue);
		        rightMotorSpeed = moveValue + rotateValue;
		    }
		 } else {  // reverse
		    if (rotateValue > 0.0) {
		        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
		        rightMotorSpeed = moveValue + rotateValue;
		    } else {
		        leftMotorSpeed = moveValue - rotateValue;
		        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
		    }
		 }
		 if (SRXDriveBaseCfg.isDriveStraightAssistEnabled) {
				// uses navx to assist in driving straight
		 } else {
			driveRightMasterMtr.set(rightMotorSpeed);
			driveLeftMasterMtr.set(leftMotorSpeed);
			SmartDashboard.putNumber("rCounts", driveRightMasterMtr.getPosition());
			SmartDashboard.putNumber("lCounts", driveLeftMasterMtr.getPosition());
			DebugLogger.log("rightEncoder:" + driveRightMasterMtr.getPosition() 
			                + "leftEncoder:" + driveLeftMasterMtr.getPosition());
			DebugLogger.data(rightMotorSpeed + "," + leftMotorSpeed + "," +
			                 driveRightMasterMtr.getSpeed() + "," + driveLeftMasterMtr.getSpeed());
		}
	}
	
	
	/**
	 * driveIndexRobot method:
	 * @parm indexDistance is in inches
	 * @parm indexTime is in seconds
	 *
	 * indexRobot uses "Magic Motion" in the Talon SRX modules to index the robot.
	 * "Magic Motion" SRX method needs cruise velocity(RPM), acceleration rate (RPM/Sec),
	 * and distance(encoder counts)
	 *
	 * The following equations for a trapezoid with 1/3 time segments are used to determine 
	 * params for "Magic Motion" move:
	 *
	 * Velocity = 1.5*(Distance / Time)
     * Accel = Decel = 4.5*(Distance / Time2)
	 *
	 * Velocity(RPM) =  (1.5*(Distance(in) / Time(sec))*60(sec/min)) / Wheel Circum(in/rev)
	 * Acceleration(RPM/sec) = ((1.5*(Distance(in) / Time(sec))*60(sec/min))
	 *							/ Wheel Circum(in/rev)) / Ta(sec)
	 * Ta should be Time/3
	 * Distance(Encoder Counts) = Distance(in) / (in/count)
	 */
	// RevB, made driveIndexRobot to receive constant calls and return a boolean state 
	public boolean SRXBaseDriveIndexRobot(double indexDistanceIn, int indexTime){
		// this is a one shot code: calculate parms for magic motion, send to SRX and start magic motion
		if (!isDriveMoving) {
			// Right drive train calculations
			double calibratedRgtDistance = indexDistanceIn * SRXDriveBaseCfg.kRgtDistanceCalibration;
			double rightDrvTrainCruiseVelSetPt =  (1.5*(calibratedRgtDistance / indexTime)*60)
											/ SRXDriveBaseCfg.kCalibratedRgtWheelCircum;
			double rightDrvTrainAccelSetPt = ((1.5*(indexDistanceIn / indexTime)*60)
										/ SRXDriveBaseCfg.kCalibratedRgtWheelCircum
										/ (indexTime * .33333));
		    rightDrvTrainTargetPosSetPt = calibratedRgtDistance / SRXDriveBaseCfg.kRgtInchesPerCount;
			
			driveRightMasterMtr.setMotionMagicCruiseVelocity(rightDrvTrainCruiseVelSetPt);
			driveRightMasterMtr.setMotionMagicAcceleration(rightDrvTrainAccelSetPt);
			
			double calibratedLftDistance = indexDistanceIn * SRXDriveBaseCfg.kLftDistanceCalibration;
			
			// Left drive train calcualtions
			double leftDrvTrainCusiseVelSetPt =  (1.5*(calibratedLftDistance / indexTime)*60)
											/ SRXDriveBaseCfg.kCalibratedLftWheelCircum;
			double leftDrvTrainAccelSetPt = ((1.5*(indexDistanceIn / indexTime)*60)
										/ SRXDriveBaseCfg.kCalibratedLftWheelCircum
										/ (indexTime * .33333));
		    leftDrvTrainTargetPosSetPt = calibratedLftDistance / SRXDriveBaseCfg.kLftInchesPerCount;
			driveLeftMasterMtr.setMotionMagicCruiseVelocity(leftDrvTrainCusiseVelSetPt);
			driveLeftMasterMtr.setMotionMagicAcceleration(leftDrvTrainAccelSetPt);
		
		// Start index
			driveRightMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveLeftMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveRightMasterMtr.set(rightDrvTrainTargetPosSetPt); 
			driveLeftMasterMtr.set(leftDrvTrainTargetPosSetPt);
		// set isDriveMoving
			isDriveMoving = true;
		}
		// If drive is moving check for end of move (?? not sure what to check)
		else if ((driveRightMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand) &&
					(driveLeftMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand)) {
			
				driveRightMasterMtr.set(0); 
				driveLeftMasterMtr.set(0);
				isDriveMoving = false;
		}
		// resend distance SRX if drive has not reached end
		else {
			driveRightMasterMtr.set(rightDrvTrainTargetPosSetPt); 
			driveLeftMasterMtr.set(leftDrvTrainTargetPosSetPt);
		}
		return isDriveMoving;
	}
	
	public void SRXBaseDriveRotate(double _RotateHeadingValue){
		
	}
		
	public void SRXBaseDriveTurn(double _TurnHeadingValue) {
		
	}
	
	public void SRXBaseDrivePowerMoveToPosition(double _PwrMoveToPositionValue) {
		
	}
	
	public boolean moveInchesAndSpeed(double distance, double speed) {
		boolean moveDone = false;
		double leftEncoderCounts = distance / SRXDriveBaseCfg.kLftInchesPerCount;
		double rightEncoderCounts = distance / SRXDriveBaseCfg.kRgtInchesPerCount;
		if ((Math.abs(driveRightMasterMtr.getEncPosition()) < rightEncoderCounts) &&
				(Math.abs(driveLeftMasterMtr.getEncPosition()) < leftEncoderCounts)) {
			setTurnAndThrottle(0, speed);
		}
		else {
			DebugLogger.log("move is done");
			moveDone = true;
		}
		return moveDone;
		
	}
	
	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void UpdateSRXDrive(){
		
		// Display SRXBaseDrive version
		SmartDashboard.putString("SRXBaseDrive-Version", VersionString);
		// Display SRX module values
		SmartDashboard.putNumber("SRXBaseDrive-Right Encoder Count", driveRightMasterMtr.getPosition());
		SmartDashboard.putNumber("SRXBaseDrive-Speed Right", driveRightMasterMtr.getSpeed());
		SmartDashboard.putNumber("SRXBaseDrive-Speed Right", driveRightMasterMtr.getClosedLoopError());
		SmartDashboard.putNumber("SRXBaseDrive-Right Bus Voltage", driveRightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Right Output Voltage", driveRightMasterMtr.getOutputVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Current Right Master", driveRightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("SRXBaseDrive-Current Right Follower", driveRightFollowerMtr.getOutputCurrent());
		
		SmartDashboard.putNumber("SRXBaseDrive-Left Encoder Count", driveLeftMasterMtr.getPosition());
		SmartDashboard.putNumber("SRXBaseDrive-Speed Left", driveLeftMasterMtr.getSpeed());
		SmartDashboard.putNumber("SSRXBaseDrive-peed Left", driveLeftMasterMtr.getClosedLoopError());
		SmartDashboard.putNumber("SRXBaseDrive-Left Bus Voltage", driveLeftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-SRXBaseDrive-Left Output Voltage", driveLeftMasterMtr.getOutputVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Current Left Master", driveLeftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("SRXBaseDrive-Current Left Follower", driveRightFollowerMtr.getOutputCurrent());
		
		if (SRXDriveBaseCfg.isHeadingModuleEnabled) {
			//Display Inertial Measurement Unit (IMU) values
			/*
			SmartDashboard.putNumber("SRXBaseDrive-ANGLE NAVX", imu.getAngle());
			SmartDashboard.putNumber("SRXBaseDrive-IMU_Yaw", imu.getYaw());
			SmartDashboard.putNumber("SRXBaseDrive-IMU_Pitch", imu.getPitch());
			SmartDashboard.putNumber("SRXBaseDrive-IMU_Roll", imu.getRoll());
			*/
		}
		// check for stall current and motor burnout
		// to do
	}
	
	public double PIDDriveStraightCorrection(){
		// This method uses the IMU and a PID equation to provide drive straight correction
		return 0.0;
	}
	
	
	public void SRXBaseDriveRotate(boolean _direction) {
		// This rotates a preset power level of 0.2; 
		// direction(true)-rotates right, direction(false)-rotates left
		if (_direction){
			driveRightMasterMtr.set(SRXDriveBaseCfg.kRotatePowerLevel * SRXDriveBaseCfg.kDriveStraightCorrection);
			driveLeftMasterMtr.set(-SRXDriveBaseCfg.kRotatePowerLevel);
		} else {
			driveRightMasterMtr.set(-SRXDriveBaseCfg.kRotatePowerLevel * SRXDriveBaseCfg.kDriveStraightCorrection);
			driveLeftMasterMtr.set(SRXDriveBaseCfg.kRotatePowerLevel);
		}
	}


	public boolean SRXBaseDriveRotateToAngle(double _rotateAngleValue){
		// this method needs work
		if (SRXDriveBaseCfg.isIMUEnabled) {
			// read the heading from the IMU module and clear heading on IMU module on first call
		} else if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			// clear encoders and set calculated encoder stop values on first call
		}
		// equations
		// powerRight = math.sign(_rotateAngleValue)*kRotatePowerLevel;
		// powerLeft = -math.sign(_rotateAngleValue)*kRotatePowerLevel;
		// check heading to _rotateAngleValue to stop or
		// check encoder stop value to stop
		return true;
	}
	
	
	
	public boolean SRXBaseDriveTurnToAngle(double _turnHeadingValue) {
		// ToDo
		return true;
	}
	
	public boolean SRXBaseDriveVelMoveToPosition(double _PwrMoveToPosiitionValue) {
		// This method moves the robot with a predetermined power level and stops at
		// the specified position value. The move will be in brake mode to stop
		// method will check that robot is stopped and set brake mode back to coast and respond
		// that move is done
		return true;
		
	}

	public void SRXBaseDriveMagicMove(double _rightCruiseVel,
										double _rightAccel,
										double _rightDistance,
										double _leftCruiseVel,
										double _leftAccel,
										double _leftDistance) {
		// This method performs a SRX magic motion command from user calculated values
		// User should note that the right drive distance needs to be corrected by kDriveStraightCorrection
		driveRightMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
		driveRightMasterMtr.setMotionMagicCruiseVelocity(_rightCruiseVel); 
		driveRightMasterMtr.setMotionMagicAcceleration(_rightAccel); 
		
		driveLeftMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
		driveLeftMasterMtr.setMotionMagicCruiseVelocity(_leftCruiseVel); 
		driveLeftMasterMtr.setMotionMagicAcceleration(_leftAccel);
		
		driveRightMasterMtr.set(_rightDistance);
		driveLeftMasterMtr.set(_leftDistance);
		}
	
	/*
	*	CloseLoop velocity Squarewave test for setting up PID values
	*/
	/*
	public void motorSquareWaveTest(bool _rightDrive, boolean _isMtrSquareWaveTestEnabled){
		if (_isMtrSquareWaveTestEnabled) {
			
			// initialize and start at low speed
			if isSqWaveFnctStartActive {
				isLowTimeActive = true;
				isSqWaveFnctStartActive = false;
				startTimeSec = getFPGATimeStamp(); // seconds
				
				// Start square wave at low speed
				If _rightDrive{
					driveRightMasterMtr.set(kSquareWaveLowSpeed);
					driveLeftMasterMtr.set(0);
				} else{
					driveLeftMasterMtr.set(kSquareWaveLowSpeed);
					driveRightMasterMtr.set(0);
				}
				
				
			}
			if isLowTimeActive {
				
				// Stay at a low speed for klowSQTime ms then switch to high speed
				If _rightDrive{
					driveRightMasterMtr.set(kSquareWaveLowSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(kSquareWaveLowSpeed);
					driveRightMasterMtr.set(0)
				}
				if (getFPGATimeStamp() - startTimeSec) > kLowSquareWaveTimeSec {
					
					// Stop high speed mode
					isLowTimeActive = false;
					
					// Set start time for high speed mode
					startTimeSec = getFPGATimeStamp();
				}
				
			} else {
				
				// Stay at a higg speed for kHighSQTime ms then switch to low speed
				if _rightDrive{
					driveRightMasterMtr.set(kRightSideHighSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(kleftSideHighSpeed);
					driveRightMasterMtr.set(0);
				}
				if (getFPGATimeStamp() - startTimeSec) > kHighSquareWaveTimeSec{

					// Stop high speed mode
					isLowTimeActive = true;
					
					// Set start time for low speed mode
					startTimeSec = getFPGATimeStamp();
				
				}		
			} 			
			SmartDashboard.putNumber("Test Low Speed", kSquareWaveLowSpeed);
			SmartDashboard.putNumber("Test high Speed", kSquareWaveHighSpeed);
			if _rightDrive{
				SmartDashboard.putNumber("Test Right Speed", driveRightMasterMtr.getSpeed());
				SmartDashboard.putNumber("Test Right Error", driveRightMasterMtr.closelooperror());
				SmartDashboard.putNumber("Test Left Speed", 0);
				SmartDashboard.putNumber("Test Left Error", 0);
			} else {
				SmartDashboard.putNumber("Test Right Speed", 0);
				SmartDashboard.putNum ber("Test Right Error", 0);
				SmartDashboard.putNumber("Test Left Speed", driveLeftMasterMtr.getSpeed());
				SmartDashboard.putNumber("Test Left Error", driveLeftMasterMtr.closelooperror());
			}
			
		} else {
			// Reset method flags for next call to motorSquareWaveTest method
			isLowTimeActive = true;
			isSqWaveFnctStartActive = true;
		}
	}
*/

}