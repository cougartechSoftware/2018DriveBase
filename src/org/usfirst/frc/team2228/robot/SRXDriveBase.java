package org.usfirst.frc.team2228.robot;
/**
* Class SRXBaseDrive
* RELEASE: 2, RevA 180117 
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
import com.ctre.CANTalon.VelocityMeasurementPeriod;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	// public class SRXDriveBase( AngleIF _angle, DistanceIF _distance)
	// AngleIF robotAngle = _angle;
	// DistanceIF robotDistance = _distance;
	
	private String VersionString = "Release 1, RevC 180108";

	// cheesy or tank motors
	private double startTime = 0;
	private CANTalon driveRightMasterMtr;
	private CANTalon driveRightFollowerMtr;
	private CANTalon driveLeftMasterMtr;
	private CANTalon driveLeftFollowerMtr;

	private RobotDrive driveStyle;
	
	private int CycleCount = 1;
	
	private double leftEncoderCounts = 0;
	private double rightDrvTrainTargetPosSetPt;
	private double leftDrvTrainTargetPosSetPt;
	private double leftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double rotationEncoderCount = 0;
	private double driveStraightDirCorrection = 0;
	private double speedRatio = 0;
	private double wheelToCenterDistanceIn = 0;
	private double outerDistanceCnts = 0;
	private double drivePerpendicularDirCorrection = 0;
	private double integral = 0;
	private double previousError = 0;
	private double previousTime = 0;
	private double previousTimeSec =0;
	private double calCorrectionFactor = 0;
	private double startStallTimerSec = 0;
	private double moveCounts = 0;
	
	//  Program flow switches
	private boolean isStallTimerActive = false;
	private boolean isStallTimerTimedOut = false;
	private boolean isVelMoveToPositionActive = false;
	private boolean isRotateToAngleActive = false;
	private boolean isTurnToAngleActive = false;
	private boolean isSRXMagicMoveActive = false;
	private boolean isLowTimeActive = false;
	private boolean isSqWaveFnctStartActive = false;
	private boolean isMovePerpendicularActive = false;
	private boolean isTestMoveForStraightCalActive = false;
	private boolean isDelayActive = false;
	private boolean isDriveTrainMoving = false;
	private boolean isConsoleDataEnabled = false;
	private boolean isLoggingDataEnabled = false;
	private boolean islogSRXDriveActive = false;
	
	String logSRXDriveString = " ";
	
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


		/*
		 * Set right/left masters and right/left followers
		 */
		// Set Right master to percentVbus mode
		driveRightMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveRightMasterMtr.enableControl();
		driveRightMasterMtr.set(0);

		// Set up right follower
		driveRightFollowerMtr.changeControlMode(TalonControlMode.Follower);

		// Set follower motor to follow master
		driveRightFollowerMtr.set(driveRightMasterMtr.getDeviceID());
		driveRightFollowerMtr.enableControl();

		// Set left master to percentVbus mode
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
			driveRightMasterMtr.configEncoderCodesPerRev(SRXDriveBaseCfg.kDriveEncoderCyclesPerRev);

			driveLeftMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveLeftMasterMtr.reverseSensor(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
			driveLeftMasterMtr.configEncoderCodesPerRev(SRXDriveBaseCfg.kDriveEncoderCyclesPerRev);
		}

		/*
		 * Setup closed-loop velocity and Setup PID values if enabled
		 */
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			setSRXSpeedModeWithFeedback();
		}

		// Create drive for WPI arcade usage
		driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr);
		driveStyle.setSafetyEnabled(false);
		/*
		 * Clear all sticky faults in drive controllers
		 */
		driveRightMasterMtr.clearStickyFaults();
		driveRightFollowerMtr.clearStickyFaults();
		driveLeftMasterMtr.clearStickyFaults();
		driveLeftFollowerMtr.clearStickyFaults();

	}
	/**
	* =======================================================================================
	* SRXBaseDrive SET METHODS
	* =======================================================================================
	*/

	public void setSRXPercentVbusMode() {
		// Set Right master to percentVbus mode
		driveRightMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveRightMasterMtr.enableControl();
		driveRightMasterMtr.set(0);

		// Set left master to percentVbus mode
		driveLeftMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveLeftMasterMtr.enableControl();
		driveLeftMasterMtr.set(0);
	}

	public void setSRXSpeedModeWithFeedback() {
		driveRightMasterMtr.setPID(SRXDriveBaseCfg.kdriveRightMstrProportionalGain,
				SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain,
				SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXDriveBaseCfg.kdriveRightMstrRampRate,
				SRXDriveBaseCfg.kdriveRightMstrIzone, SRXDriveBaseCfg.kdriveRightMstrProfile);

		driveLeftMasterMtr.setPID(SRXDriveBaseCfg.kdriveLeftMstrProportionalGain,
				SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain,
				SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXDriveBaseCfg.kdriveLeftMstrRampRate,
				SRXDriveBaseCfg.kdriveleftMstrIzone, SRXDriveBaseCfg.kdriveLeftMstrProfile);

		// set controlMode to speed with feedback
		// Note speed is in RPM, thus -1 to 1 input needs to be multiplied by
		// max RPM
		driveRightMasterMtr.changeControlMode(TalonControlMode.Speed);
		driveLeftMasterMtr.changeControlMode(TalonControlMode.Speed);

		// The following compensates for battery voltage - 50% output would be
		// %50 of 11 volts
		// driveRightMasterMtr.setNominalClosedLoopVoltage(11.0);

		// cnts / 4096 cnts/rev(magnetic encoder); 100/4096= 2.4%; 8.7 degrees
		driveRightMasterMtr.setAllowableClosedLoopErr(SRXDriveBaseCfg.kClosedLoopErr);
		driveLeftMasterMtr.setAllowableClosedLoopErr(SRXDriveBaseCfg.kClosedLoopErr);

		// Sample period in ms from supported sample periods-default 100ms
		// period/64 sample window
		driveRightMasterMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_25Ms);
		driveLeftMasterMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_25Ms);

		// Number if samples in a rolling average
		driveRightMasterMtr.SetVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample);
		driveLeftMasterMtr.SetVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample);

		// Activate closed-Loop velocity
		driveRightMasterMtr.set(0);
		driveLeftMasterMtr.set(0);

	}

	public void setRightEncPositionToZero() {
		driveRightMasterMtr.setEncPosition(0);
	}

	public void setLeftEncPositionToZero() {
		driveLeftMasterMtr.setEncPosition(0);

	}
	
	public void setRightPositionToZero() {
		driveRightMasterMtr.setPosition(0);
	}

	public void setLeftPositionToZero() {
		driveLeftMasterMtr.setPosition(0);

	}

	public void setBrakeMode(boolean isBrakeEnabled) {
		driveRightMasterMtr.enableBrakeMode(isBrakeEnabled);
		driveRightFollowerMtr.enableBrakeMode(isBrakeEnabled);
		driveLeftMasterMtr.enableBrakeMode(isBrakeEnabled);
		driveLeftFollowerMtr.enableBrakeMode(isBrakeEnabled);
	}
	
	public void setStopMotors(){
		driveRightMasterMtr.set(0);
		driveLeftMasterMtr.set(0);
	}
	public void setEnableConsoleData(boolean _consoleData){
		isConsoleDataEnabled = _consoleData;
	}
	public void setEnableLoggingData(boolean _loggingData){
		isLoggingDataEnabled = _loggingData;
	}
	public void setClearActionFlags() {
		isStallTimerActive = false;
		isStallTimerTimedOut = false;
		isVelMoveToPositionActive = false;
		isRotateToAngleActive = false;
		isTurnToAngleActive = false;
		isSRXMagicMoveActive = false;
		isLowTimeActive = false;
		isSqWaveFnctStartActive = false;
		isMovePerpendicularActive = false;
		isTestMoveForStraightCalActive = false;
		isDelayActive = false;
		isDriveTrainMoving = false;
		islogSRXDriveActive = false;
	}

	/**
	* =======================================================================================
	* STATUS METHODS
	* =======================================================================================
	*/
	
		
	public void DisplayChangeParmeters() {
		SmartDashboard.putNumber("Right Correction Factor", SRXDriveBaseCfg.kDriveStraightCorrection);
	}
	
	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void UpdateSRXDriveDataDisplay() {

		// Display SRXBaseDrive version
		SmartDashboard.putString("SRXBaseDrive-Version", VersionString);
		// Display SRX module values
		SmartDashboard.putNumber("BaseDrive-Right Bus Voltage", driveRightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Right Output Voltage", driveRightMasterMtr.getOutputVoltage());
		SmartDashboard.putNumber("BaseDrive-Current Right Master", driveRightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Current Right Follower", driveRightFollowerMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Left Bus Voltage", driveLeftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Left Output Voltage", driveLeftMasterMtr.getOutputVoltage());
		SmartDashboard.putNumber("BaseDrive-Current Left Master", driveLeftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Current Left Follower", driveRightFollowerMtr.getOutputCurrent());

		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("BaseDrive-Right Encoder Count", driveRightMasterMtr.getEncPosition());
			SmartDashboard.putNumber("BaseDrive-Speed Right", driveRightMasterMtr.getSpeed());
			SmartDashboard.putNumber("BaseDrive-Left Encoder Count", driveLeftMasterMtr.getEncPosition());
			SmartDashboard.putNumber("BaseDrive-Speed Left", driveLeftMasterMtr.getSpeed());
		}

		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			SmartDashboard.putNumber("BaseDrive-Speed Right ClosedLoopErr",
					driveRightMasterMtr.getClosedLoopError());
			SmartDashboard.putNumber("BaseDrive-Speed Left ClosedLoopErr", driveLeftMasterMtr.getClosedLoopError());
		}

		if (SRXDriveBaseCfg.isHeadingModuleEnabled) {
			// Display Inertial Measurement Unit (IMU) values
			/*
			 * SmartDashboard.putNumber("BaseDrive-ANGLE NAVX",
			 * imu.getAngle()); SmartDashboard.putNumber("SRXBaseDrive-IMU_Yaw",
			 * imu.getYaw()); SmartDashboard.putNumber("SRXBaseDrive-IMU_Pitch",
			 * imu.getPitch());
			 * SmartDashboard.putNumber("SRXBaseDrive-IMU_Roll", imu.getRoll());
			 */
		}
		// check for stall current and motor burnout
		// to do
	}

	public void logSRXDriveData(){
		if (isLoggingDataEnabled){
			if (!islogSRXDriveActive){
				islogSRXDriveActive = true;
				logSRXDriveString = "Right Bus Voltage,Right Output Voltage,Right Master Current,Right Follower Current,Left Bus Voltage,Left Output Voltage,Left Master Current,Left Follower Current,Right Encoder Count,Left Encoder Count";
				// Log data
				DebugLogger.data(logSRXDriveString);
			} else {
				logSRXDriveString = String.format("%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f", 
										driveRightMasterMtr.getBusVoltage(), 
										driveRightMasterMtr.getOutputVoltage(),
										driveRightMasterMtr.getOutputCurrent(),
										driveRightFollowerMtr.getOutputCurrent(),
										driveRightFollowerMtr.getOutputCurrent(),
										driveLeftMasterMtr.getBusVoltage(),
										driveLeftMasterMtr.getOutputVoltage(),
										driveLeftMasterMtr.getOutputCurrent(),
										driveRightMasterMtr.getEncPosition(),
										driveLeftMasterMtr.getEncPosition() );
				// Log data
				DebugLogger.data(logSRXDriveString);
			}
		} else {
			islogSRXDriveActive = false;
		}
	} 

	public double getRightEncoder(){
		return driveRightMasterMtr.getEncPosition();
	}
	
	public double getRightEncoderPosition() {
		if (SRXDriveBaseCfg.isRightEncoderSensorReversed){
			return -driveRightMasterMtr.getEncPosition();
		} else{
			return driveRightMasterMtr.getEncPosition();
		}
	}
	public double getRightClosedLoopPosition(){
		return driveRightMasterMtr.getPosition();
	}
	public double getRightMstrMtrCurrent() {
		return driveRightMasterMtr.getOutputCurrent();
	}

	public double getRightFollowerMtrCurrent() {
		return driveRightFollowerMtr.getOutputCurrent();
	}

	public double getRightClosdedLoopVelocity() {
		return driveRightMasterMtr.getSpeed();
	}

	public double getRightCloseLoopError() {
		return driveRightMasterMtr.getClosedLoopError();
	}
	public double getLeftEncoder() {
		return driveLeftMasterMtr.getEncPosition();
	}
	public double getLeftEncoderPosition() {
		if (SRXDriveBaseCfg.isLeftEncoderSensorReversed){
			return -driveLeftMasterMtr.getEncPosition();
		} else{
			return driveLeftMasterMtr.getEncPosition();
		}
	}
	public double getLeftClosedLoopPosition(){
		return driveLeftMasterMtr.getPosition();
	}
	public double getLeftMstrMtrCurrent() {
		return driveLeftMasterMtr.getOutputCurrent();
	}

	public double getLeftFollowerMtrCurrent() {
		return driveLeftFollowerMtr.getOutputCurrent();
	}

	public double getLeftClosedLoopVelocity() {
		return driveLeftMasterMtr.getSpeed();
	}

	public double getLeftCloseLoopError() {
		return driveLeftMasterMtr.getClosedLoopError();
	}

	public double getBusVoltage() {
		return driveLeftMasterMtr.getBusVoltage();
	}
	
	public boolean isDriveMoving() {
		return isDriveTrainMoving;
	}
	/**
	* =======================================================================================
	* TELEOP METHODS
	* =======================================================================================
	*/
	/*
	 * Note: left drive is master drive axis for the robot - the right drive
	 * will be modified for driving straight
	 *
	 * NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the
	 * motor bus voltage). Motion command with closed loop reflect speed level
	 * (-1 to 1) * (top motor RPM)
	 */
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		rightCmdLevel = _rightCMDLevel;
		leftCmdLevel = _leftCMDLevel;
		isDriveTrainMoving =((rightCmdLevel > 0) || (leftCmdLevel > 0))? true : false;
		rightCmdLevel *= SRXDriveBaseCfg.kDriveStraightCorrection;
		if (getIsStallConditionTimedOut()) {
			rightCmdLevel = 0;
			leftCmdLevel = 0;	
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		driveRightMasterMtr.set(rightCmdLevel);
		driveLeftMasterMtr.set(leftCmdLevel);
	}

	/*
	 * WPI throttle and turn commands This method uses WPI library methods to
	 * drive the robot with a throttle and turn input. Drives were set up by:
	 * driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
	 * throttle would be the game controller Y-axis(joystick fwd/rev) and turn
	 * would be game conctroller X-axis(joystick left/right)
	 *
	 * NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
	 * to 1) * (the motor bus voltage). The speed is determined by this power
	 * level and the load to the motor.
	 */
	public void WPISetThrottleTurn(double throttleValue, double turnValue) {
		// Check stall condition
		isDriveTrainMoving =((throttleValue > 0) || (turnValue > 0))? true : false;
		if (getIsStallConditionTimedOut()) {
			throttleValue = 0;
			turnValue = 0;
		}
		driveStyle.arcadeDrive(throttleValue, turnValue, false);	
	}

	/*
	 * setThrottleTurn is both open loop and closed loop control with drive
	 * straight/drive perpendicular correction
	 */
	public void setThrottleTurn(double _throttleValue, double _turnValue, boolean _isDrivingPerpendicular) {
		isDriveTrainMoving =((_throttleValue > 0) || (_turnValue > 0))? true : false;
		// Check stall condition
		if (getIsStallConditionTimedOut()) {
			_throttleValue = 0;
			_turnValue = 0;
		}

		// The turn joystick needs to be in its zero position to have drive
		// direction assist
		if ((Math.abs(_turnValue) < SRXDriveBaseCfg.kTurnValueDeadBand)) {
			_turnValue = 0;
		}

		if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && _turnValue == 0) {

			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			// driveStraightDirCorrection = robotAngle.getAngleCorrection();
			driveStraightDirCorrection = 0;
			leftCmdLevel = _throttleValue + _turnValue + driveStraightDirCorrection;
			rightCmdLevel = ((_throttleValue - _turnValue) * SRXDriveBaseCfg.kDriveStraightCorrection)
								- driveStraightDirCorrection;

		} else {
			leftCmdLevel = _throttleValue + _turnValue;
			rightCmdLevel = ((_throttleValue - _turnValue) * SRXDriveBaseCfg.kDriveStraightCorrection);
		}

		// This converts cmd to speed in RPM; range -TopRPM to TopRPM
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		// Output left/right command levels
		driveLeftMasterMtr.set(leftCmdLevel);
		driveRightMasterMtr.set(rightCmdLevel);
		if (isConsoleDataEnabled){
			System.out.printf("Throttle:%-8.3f===Turn:%-8.3f===RightCmd:%-8.3f===LeftCmd:%-8.3f===RightCurrent:%-8.3f===LeftCurrent:%-8.3f%n", 
									_throttleValue, 
									_turnValue,
									rightCmdLevel,
									leftCmdLevel,
									getRightMstrMtrCurrent(),
									getLeftMstrMtrCurrent());
		}
	}
	
	/*
	 * If robot blocked by another robot Fast turn turns robot 180 deg to escape
	 */
	public void setFastTurn(boolean _isFastTurnRight) {
		// future TODO
	}

	/*
	 * This method flags a robot in a stall condition longer than stall time
	 */
	public boolean getIsStallConditionTimedOut() {
		// Check if motors are in a stall current state and kill drives if left there too long
		if (((Math.abs(driveRightMasterMtr.getOutputCurrent()) > SRXDriveBaseCfg.kStallCurrent) 
			||  Math.abs(driveLeftMasterMtr.getOutputCurrent()) > SRXDriveBaseCfg.kStallCurrent)) {
				if (!isStallTimerActive) {
					startStallTimerSec  = Timer.getFPGATimestamp();
					isStallTimerActive = true;
				} else {
					if ((Timer.getFPGATimestamp() - startStallTimerSec) > SRXDriveBaseCfg.kStallTimeSec) {
						isStallTimerTimedOut = true;
					}
				}
		} else {
			isStallTimerActive = false;
			isStallTimerTimedOut = false;
		}
		return isStallTimerTimedOut;
	}


	/**
	* =======================================================================================
	* AUTONOMOUG METHODS
	* =======================================================================================
	*/
	public boolean velMoveToPosition(double _MoveToPositionIn, double _MoveToPositionPwrLevel, boolean _isCascadeMove) {
		// This method moves the robot with a predetermined power level and stops at
		// the specified position value. The move will be in brake mode to stop
		// method will check that robot is stopped and set brake mode back to coast and respond
		// that move is done
		if (!isVelMoveToPositionActive) {
			isVelMoveToPositionActive = true;
			isDriveTrainMoving = true;
			driveRightMasterMtr.setEncPosition(0);
			driveLeftMasterMtr.setEncPosition(0);
			setBrakeMode(true);
			moveCounts = (Math.abs(_MoveToPositionIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn)
							- SRXDriveBaseCfg.kRobotCoastToStopCounts;
			leftCmdLevel = (Math.signum(_MoveToPositionIn)*_MoveToPositionPwrLevel);
			rightCmdLevel = (Math.signum(_MoveToPositionIn)*_MoveToPositionPwrLevel) * SRXDriveBaseCfg.kDriveStraightCorrection;
			
		} else {
			if (getLeftEncoderPosition() >= moveCounts) {
				if (_isCascadeMove) {
					isVelMoveToPositionActive = false;	
					System.out.println("cascade move foward done");
				} else {
						// Apply power level (.05) in opposite direction for 1 second to brake
						rightCmdLevel = -(Math.signum(_MoveToPositionIn)*0.05);
						leftCmdLevel = -(Math.signum(_MoveToPositionIn)*0.05);
					if (!delay(1)) {
						isVelMoveToPositionActive = false;
						isDriveTrainMoving = false;
						setBrakeMode(false);
						rightCmdLevel = 0;
						leftCmdLevel = 0;
					}
				}
			}
			
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		driveRightMasterMtr.set(rightCmdLevel) ;
		driveLeftMasterMtr.set(leftCmdLevel);
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f%n",
									moveCounts,
									getLeftEncoderPosition(), 
									getRightEncoderPosition());
		}
		return isVelMoveToPositionActive;
		
	}
/*
	public boolean movePerpendicularToStop(double _movePerpendicularPowerLevel, double _movePerpendicularStopIn) {
		if (!isMovePerpendicularActive) {
			isMovePerpendicularActive = true;
			rightCmdLevel = _movePerpendicularPowerLevel;
			leftCmdLevel = _movePerpendicularPowerLevel;	
		} else if (robotDistance.getPerpendicularStop(_movePerpendicularStopIn)){
			rightCmdLevel=0;
			leftCmdLevel=0;
			isMovePerpendicularActive=false;
		} else{
			//drivePerpendicularDirCorrection = robotDistance.getPerpendicularCorrection();
			leftCmdLevel += drivePerpendicularDirCorrection;
			rightCmdLevel -= drivePerpendicularDirCorrection;
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled)	{
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		driveRightMasterMtr.set(rightCmdLevel);
		driveLeftMasterMtr.set(leftCmdLevel);
		return isMovePerpendicularActive;
	}
*/
	public boolean rotateToAngle(double _rotateToAngle, double _rotatePowerLevel) {
		// direction(true)-rotates right, direction(false)-rotates left
		if (!isRotateToAngleActive) {
			isRotateToAngleActive = true;
			isDriveTrainMoving = true;
			driveRightMasterMtr.setEncPosition(0);
			driveLeftMasterMtr.setEncPosition(0);
			rightCmdLevel = -Math.signum(_rotateToAngle) * _rotatePowerLevel; 
			leftCmdLevel = Math.signum(_rotateToAngle) * _rotatePowerLevel;
			
			// (C = PI*D) * (angle as a fraction of C)
			rotationEncoderCount = Math.PI*(SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn * (_rotateToAngle / 360); 
		} else if (driveLeftMasterMtr.getEncPosition() >= rotationEncoderCount) {
	
			// Apply power level (.05) in opposite direction for 1 second to brake
				rightCmdLevel = (Math.signum(_rotateToAngle)*0.05);
				leftCmdLevel = -(Math.signum(_rotateToAngle)*0.05);
			if (!delay(1)) {
				isRotateToAngleActive = false;
				isDriveTrainMoving = false;
				setBrakeMode(false);
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}		
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		driveRightMasterMtr.set(rightCmdLevel);
		driveLeftMasterMtr.set(leftCmdLevel);
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f%n",
									rotationEncoderCount,
									getLeftEncoderPosition(), 
									getRightEncoderPosition());
		}
		return isRotateToAngleActive;
	} 

	public boolean turnByEncoderToAngle(double _turnAngleDeg, double _turnRadiusIn, double _turnPowerLevel, boolean _isDirectionReverse, boolean _isCascadeTurn ) {
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			isDriveTrainMoving = true;
			driveRightMasterMtr.setEncPosition(0);
			driveLeftMasterMtr.setEncPosition(0);
			wheelToCenterDistanceIn = (SRXDriveBaseCfg.kTrackWidthIn / 2);
			speedRatio =(_turnRadiusIn + wheelToCenterDistanceIn) / (_turnRadiusIn - wheelToCenterDistanceIn);
			if (_turnAngleDeg > 0) {
				rightCmdLevel = (_turnPowerLevel);
				leftCmdLevel = (_turnPowerLevel * speedRatio);
		
			} else {
				rightCmdLevel = (_turnPowerLevel * speedRatio);
				leftCmdLevel = (_turnPowerLevel);
			}
			
			// Convert turn distance in inches to encoder counts
			if (_turnAngleDeg > 0) {
				outerDistanceCnts = 2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) *	SRXDriveBaseCfg.kLeftEncoderCountsPerIn;
			} else {
				outerDistanceCnts = 2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn;
			}
		} else {
			if (_isCascadeTurn) {
				isTurnToAngleActive = false;
			} else if ((_turnAngleDeg >= 0 && (getLeftEncoderPosition() > outerDistanceCnts))
					|| (_turnAngleDeg <= 0 && (getRightEncoderPosition() > outerDistanceCnts))) {
						
				// Apply power level (.05) in opposite direction for 1 second to brake
				rightCmdLevel = -(Math.signum(_turnAngleDeg) * 0.05);
				leftCmdLevel = -(Math.signum(_turnAngleDeg) * 0.05);
				if (!delay(1)) {
					isTurnToAngleActive = false;
					System.out.println("Active flag" + isTurnToAngleActive);
					isDriveTrainMoving = false;
					setBrakeMode(false);
					rightCmdLevel = 0;
					leftCmdLevel = 0;	
				}
			}
		}
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		driveRightMasterMtr.set(rightCmdLevel);
		driveLeftMasterMtr.set(leftCmdLevel);
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f%n",
									outerDistanceCnts,
									getLeftEncoderPosition(), 
									getRightEncoderPosition());
		}
		return isTurnToAngleActive;
	}
	
	
	
	/**
	* =======================================================================================
	* SRXDriveBase TEST METHODS
	* =======================================================================================
	*/
	public void testMotorSquareWave(boolean _isMtrSquareWaveTestEnabled, boolean _isTestForRightDrive) {
		if (_isMtrSquareWaveTestEnabled && SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			double startTimeSec = 0;
			// initialize and start at low speed
			if (!isSqWaveFnctStartActive) {
				isLowTimeActive = true;
				isSqWaveFnctStartActive = true;
				startTimeSec = Timer.getFPGATimestamp(); // seconds

				// Start square wave at low speed
				if (_isTestForRightDrive) {
					driveRightMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveRightMasterMtr.set(0);
				}
			}
			if (isLowTimeActive) {

				// Stay at a low speed for klowSQTime ms then switch to high
				// speed
				if (_isTestForRightDrive) {
					driveRightMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveRightMasterMtr.set(0);
				}
				if ((Timer.getFPGATimestamp() - startTimeSec) > SRXDriveBaseCfg.kSquareWaveLowerSpeed) {

					// Stop high speed mode
					isLowTimeActive = false;

					// Set start time for high speed mode
					startTimeSec = Timer.getFPGATimestamp();
				}

			} else {

				// Stay at a high speed for kHighSQTime ms then switch to low
				// speed
				if (_isTestForRightDrive) {
					driveRightMasterMtr.set(SRXDriveBaseCfg.kSquareWaveHigherSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(SRXDriveBaseCfg.kSquareWaveHigherSpeed);
					driveRightMasterMtr.set(0);
				}
				if ((Timer.getFPGATimestamp() - startTimeSec) > SRXDriveBaseCfg.kSquareWaveHigherSpeed) {
					// Stop high speed mode
					isLowTimeActive = true;

					// Set start time for low speed mode
					startTimeSec = Timer.getFPGATimestamp();

				}
			}
			SmartDashboard.putNumber("Test Low Speed(RPM)", SRXDriveBaseCfg.kSquareWaveLowerSpeed);
			SmartDashboard.putNumber("Test high Speed(RPM)", SRXDriveBaseCfg.kSquareWaveHigherSpeed);
			if (_isTestForRightDrive) {
				SmartDashboard.putNumber("Test Right Speed", driveRightMasterMtr.getSpeed());
				SmartDashboard.putNumber("Test Right Error", driveRightMasterMtr.getClosedLoopError());
				SmartDashboard.putNumber("Test Left Speed", 0);
				SmartDashboard.putNumber("Test Left Error", 0);
			} else {
				SmartDashboard.putNumber("Test Right Speed", 0);

				SmartDashboard.putNumber("Test Right Error", 0);
				SmartDashboard.putNumber("Test Left Speed", driveLeftMasterMtr.getSpeed());
				SmartDashboard.putNumber("Test Left Error", driveLeftMasterMtr.getClosedLoopError());
			}

		} else {
			// Reset method flags for next call to motorSquareWaveTest method
			isLowTimeActive = false;
			isSqWaveFnctStartActive = false;
			driveLeftMasterMtr.set(0);
			driveRightMasterMtr.set(0);
		}
	}

	public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel){
		if (!isTestMoveForStraightCalActive){
			isTestMoveForStraightCalActive = true;
			CycleCount = 0;
			leftEncoderCounts = _testDistanceIn / SRXDriveBaseCfg.kLftInchesPerCount;
			leftCmdLevel = _pwrLevel;
			rightCmdLevel = _pwrLevel;
			isDriveTrainMoving = true;
			setRightEncPositionToZero();
			setLeftEncPositionToZero();
		} else if (getLeftEncoderPosition() >= leftEncoderCounts) {
			
			// Apply power level (.1) in oposite direction for 1 second to brake
			rightCmdLevel = -0.05;
			leftCmdLevel = -0.05;
			if (!delay(1)) {
				isTestMoveForStraightCalActive = false;
				isDriveTrainMoving = false;
				setBrakeMode(false);
				rightCmdLevel = 0;
				leftCmdLevel = 0;
			}	
		}
		calCorrectionFactor = getLeftEncoderPosition() / getRightEncoderPosition();
		driveLeftMasterMtr.set(leftCmdLevel);
		driveRightMasterMtr.set(rightCmdLevel);
		
		if (isLoggingDataEnabled) {
			String outputString = String.format("%8.0f,%8.0f,%8.0f,%8.4f", 
										leftEncoderCounts, 
										getLeftEncoderPosition(), 
										getRightEncoderPosition(),
										calCorrectionFactor);
			// Log data
			DebugLogger.data(outputString);
		}
		//Print on console data
		if (isConsoleDataEnabled){
			System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f ===Correction:%-8.4f%n", 
								leftEncoderCounts, 
								getLeftEncoderPosition(), 
								getRightEncoderPosition(),
								calCorrectionFactor);
		}
		
		return isTestMoveForStraightCalActive;
	} 
	
	
	public boolean delay(double _seconds){
		if (!isDelayActive) {
			isDelayActive = true;
			startTime = Timer.getFPGATimestamp();
		} else if (Timer.getFPGATimestamp() >= (startTime + _seconds)){
			isDelayActive = false;
		}
		return isDelayActive;
	}
	
	
	/**
	* =======================================================================================
	* INDEX AND PROFILE COMMANDS
	* =======================================================================================
	*/
	public boolean magicMove(double _rightCruiseVel, double _rightAccel, double _rightDistance, double _leftCruiseVel,
			double _leftAccel, double _leftDistance) {
		// This method performs a SRX magic motion command from user calculated
		// values
		// User should note that the right drive distance needs to be corrected
		// by kDriveStraightCorrection
		if (!isSRXMagicMoveActive) {
			isSRXMagicMoveActive = true;
			driveRightMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveRightMasterMtr.setMotionMagicCruiseVelocity(_rightCruiseVel);
			driveRightMasterMtr.setMotionMagicAcceleration(_rightAccel);

			driveLeftMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveLeftMasterMtr.setMotionMagicCruiseVelocity(_leftCruiseVel);
			driveLeftMasterMtr.setMotionMagicAcceleration(_leftAccel);

			_rightDistance = _rightDistance * SRXDriveBaseCfg.kDriveStraightCorrection;
			_leftDistance = _leftDistance;
		} else {
			if (getRightEncoderPosition() >= _leftDistance) {
				isSRXMagicMoveActive = false;
				_rightDistance = 0;
				_leftDistance = 0;
			}
		}
		driveRightMasterMtr.set(_rightDistance);
		driveLeftMasterMtr.set(_leftDistance);
		return isSRXMagicMoveActive;
	}
	
	/**
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
	 */
	// RevB, made driveIndexRobot to receive constant calls and return a boolean
	// state
	public boolean SRXBaseDriveIndexRobot(double indexDistanceIn, int indexTime) {
		// this is a one shot code: calculate parms for magic motion, send to
		// SRX and start magic motion
		if (!isDriveTrainMoving) {
			// Right drive train calculations
			double calibratedRgtDistance = indexDistanceIn * SRXDriveBaseCfg.kRgtDistanceCalibration;
			double rightDrvTrainCruiseVelSetPt = (1.5 * (calibratedRgtDistance / indexTime) * 60)
					/ SRXDriveBaseCfg.kMeasuredRgtWheelCircum;
			double rightDrvTrainAccelSetPt = ((1.5 * (indexDistanceIn / indexTime) * 60)
					/ SRXDriveBaseCfg.kMeasuredRgtWheelCircum / (indexTime * .33333));
			rightDrvTrainTargetPosSetPt = calibratedRgtDistance / SRXDriveBaseCfg.kRgtInchesPerCount;

			driveRightMasterMtr.setMotionMagicCruiseVelocity(rightDrvTrainCruiseVelSetPt);
			driveRightMasterMtr.setMotionMagicAcceleration(rightDrvTrainAccelSetPt);

			double calibratedLftDistance = indexDistanceIn * SRXDriveBaseCfg.kLftDistanceCalibration;

			// Left drive train calcualtions
			double leftDrvTrainCusiseVelSetPt = (1.5 * (calibratedLftDistance / indexTime) * 60)
					/ SRXDriveBaseCfg.kMeasuredLftWheelCircum;
			double leftDrvTrainAccelSetPt = ((1.5 * (indexDistanceIn / indexTime) * 60)
					/ SRXDriveBaseCfg.kMeasuredLftWheelCircum / (indexTime * .33333));
			leftDrvTrainTargetPosSetPt = calibratedLftDistance / SRXDriveBaseCfg.kLftInchesPerCount;
			driveLeftMasterMtr.setMotionMagicCruiseVelocity(leftDrvTrainCusiseVelSetPt);
			driveLeftMasterMtr.setMotionMagicAcceleration(leftDrvTrainAccelSetPt);

			// Start index
			driveRightMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveLeftMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveRightMasterMtr.set(rightDrvTrainTargetPosSetPt);
			driveLeftMasterMtr.set(leftDrvTrainTargetPosSetPt);
		}
		// If drive is moving check for end of move (?? not sure what to check)
		else if ((driveRightMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand)
				&& (driveLeftMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand)) {

			driveRightMasterMtr.set(0);
			driveLeftMasterMtr.set(0);
		}
		// resend distance SRX if drive has not reached end
		else {
			driveRightMasterMtr.set(rightDrvTrainTargetPosSetPt);
			driveLeftMasterMtr.set(leftDrvTrainTargetPosSetPt);
		}
		return isDriveTrainMoving;
	}
	
}