package frc.robot.quail;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class CoaxSwerveModule {
	public Vec2d positionFromCenter;
	/** The motor controlling the module's movement. */
	public final WPI_TalonFX mDrivingMotor;
	/** The motor controlling the module's rotation. */
	public final WPI_TalonFX mSteeringMotor;
	/** The can coder measuring the module's absolute rotaiton. */
	private final WPI_CANCoder mCanCoder;
	/**
	 * The can coder's rotational offset. This value must be manually set through
	 * phoenix tuner.
	 */
	public final double kCanOffset;
	/** The direction of rotation relative to the center of the drivetrain. */
	private final double rotDireciton = this.positionFromCenter.getAngle() - 90;
	/** The steering motor rotation measured in radians. */
	private double motorRotation;

	private double steeringRatio;

	public double[] kPID;

	private int kMS_DELAY;

	public CoaxSwerveModule(int driveMotor,
			int steeringMotor,
			int canCoder,
			double canCoderOffset,
			Vec2d positionFromCenter,
			double steeringRatio,
			double drivingRatio,
			double wheelDiameter,
			int MS_DELAY,
			double[] PID) {
		this.mDrivingMotor = new WPI_TalonFX(driveMotor);
		this.mSteeringMotor = new WPI_TalonFX(driveMotor);
		this.mCanCoder = new WPI_CANCoder(canCoder);
		this.kCanOffset = canCoderOffset;
		this.kMS_DELAY = MS_DELAY;
		kPID = PID;
	}

	public void init() {
		// Reset the steering motor.
		this.mSteeringMotor.configFactoryDefault();
		this.mDrivingMotor.configFactoryDefault();
		int DRIVING_PID_ID = this.mDrivingMotor.getDeviceID();
		int STEERING_PID_ID = this.mSteeringMotor.getDeviceID();
		// Miscellaneous settings.
		this.mSteeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, STEERING_PID_ID,
				kMS_DELAY);
		this.mSteeringMotor.setSensorPhase(true);
		this.mSteeringMotor.setInverted(TalonFXInvertType.CounterClockwise);
		this.mSteeringMotor.configNominalOutputForward(0d, kMS_DELAY);
		this.mSteeringMotor.configNominalOutputReverse(0d, kMS_DELAY);
		this.mSteeringMotor.configPeakOutputForward(1d, kMS_DELAY);
		this.mSteeringMotor.configPeakOutputReverse(-1d, kMS_DELAY);
		this.mSteeringMotor.configAllowableClosedloopError(0, STEERING_PID_ID, kMS_DELAY);

		// PID tune the steering motor.
		this.mSteeringMotor.selectProfileSlot(STEERING_PID_ID, 0);
		this.mSteeringMotor.config_kF(STEERING_PID_ID, this.kPID[0], kMS_DELAY);
		this.mSteeringMotor.config_kP(STEERING_PID_ID, this.kPID[1], kMS_DELAY);
		this.mSteeringMotor.config_kI(STEERING_PID_ID, this.kPID[2], kMS_DELAY);
		this.mSteeringMotor.config_kD(STEERING_PID_ID, this.kPID[3], kMS_DELAY);

		this.mDrivingMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DRIVING_PID_ID,
				kMS_DELAY);
		this.mDrivingMotor.setSensorPhase(true);
		this.mDrivingMotor.setInverted(false);
		this.mDrivingMotor.configNominalOutputForward(0d, kMS_DELAY);
		this.mDrivingMotor.configNominalOutputReverse(0d, kMS_DELAY);
		this.mDrivingMotor.configPeakOutputForward(1d, kMS_DELAY);
		this.mDrivingMotor.configPeakOutputReverse(-1d, kMS_DELAY);
		this.mDrivingMotor.configAllowableClosedloopError(0, DRIVING_PID_ID, kMS_DELAY);

		// PID tune the steering motor.
		this.mDrivingMotor.selectProfileSlot(DRIVING_PID_ID, 0);
		this.mDrivingMotor.config_kF(DRIVING_PID_ID, this.kPID[0], kMS_DELAY);
		this.mDrivingMotor.config_kP(DRIVING_PID_ID, this.kPID[1], kMS_DELAY);
		this.mDrivingMotor.config_kI(DRIVING_PID_ID, this.kPID[2], kMS_DELAY);
		this.mDrivingMotor.config_kD(DRIVING_PID_ID, this.kPID[3], kMS_DELAY);
		System.out.println("done config");

		this.mDrivingMotor.configMotionAcceleration(512 * 9);
		this.mDrivingMotor.configMotionCruiseVelocity(15 * 9 * 2048);
		this.mDrivingMotor.configMotionSCurveStrength(5);
		this.mDrivingMotor.configNeutralDeadband(0.01);

		this.mCanCoder.configMagnetOffset(this.kCanOffset, kMS_DELAY);
		
	}

	public void reset() {
		this.mSteeringMotor.setSelectedSensorPosition(
			this.mCanCoder.getAbsolutePosition() * Constants.DEG_TO_TICK * this.steeringRatio);
		this.motorRotation = this.mCanCoder.getAbsolutePosition() * Constants.RAD_TO_DEG;
	}

	public void zero() {
		this.reset();
		this.mSteeringMotor.set(TalonFXControlMode.Position, 0);
		this.mDrivingMotor.setSelectedSensorPosition(0);
		this.motorRotation = 0d;
	}

	/**
	 * The motor rotation function. Simply rotates the steering motor in the
	 * direction of
	 * the passed vector, and sets the driving motor's speed to the length of the
	 * vector.
	 * The steering motor will take the shortest path of rotation by rotating a
	 * maximum of
	 * 180° in either direction. Calculations by Alex Green and Marcus Kauffman.
	 * 
	 * @param vec the vector representing the module's movement
	 * @notdeprecated in favor of {@link #setMotion(Vec2d) setMotion}.
	 * @see <a href="https://www.desmos.com/calculator/dgkniftpn6">Alex's
	 *      Calculations</a>
	 */

	public void setAngle(Vec2d vec) {
		// Get the angle of the passed vector.
		double angle = vec.getAngle();

		// Only rotate if the angle isn't NaN.
		if (!Double.isNaN(angle)) {
			// The steering motor's rotation bounded between 0 and 2π.
			double motorAngle = this.motorRotation % Constants.TWO_PI;
			// The distance between the motor's current rotation and the passed rotation.
			double angleDif = motorAngle - angle;
			// An overcomplicated way of getting the shortest distance to to the passed
			// vector.
			this.motorRotation -= angleDif
					+ (Math.abs(angleDif) > Math.PI ? (motorAngle > angle ? -Constants.TWO_PI : Constants.TWO_PI) : 0d)
					+ Constants.PI_OVER_TWO;
		}

		// Update the two motor's with the new values.
		this.mSteeringMotor.set(TalonFXControlMode.Position,
				(this.motorRotation) * Constants.RAD_TO_TICK * this.steeringRatio);
	}

    /// sets the speed and angle of the module.
    public void setMotion(Vec2d vec) {
        this.setAngle(vec);
        this.mDrivingMotor.set(TalonFXControlMode.PercentOutput, vec.getLength());
    }


    
	/**
	 * Getter for the drive motor's rotation direction.
	 * 
	 * @return The drive motor
	 */
	public double getRotationDirection()
	{
		return this.rotDireciton;
	}

	/**
	 * Getter for the drive motor.
	 * 
	 * @return The drive motor
	 */
     	public WPI_TalonFX getDriveMotor()
	{
		return this.mDrivingMotor;
	}

	/**
	 * Getter for the steering motor.
	 * 
	 * @return The steering motor.
	 */
	public WPI_TalonFX getSteeringMotor()
	{
		return this.mSteeringMotor;
	}

	/**
	 * Getter for the can coder.
	 * 
	 * @return The can coder
	 */
	public WPI_CANCoder getCanCoder()
	{
		return this.mCanCoder;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.mSteeringMotor.getDeviceID() + ", Driving Motor ID = " + this.mDrivingMotor.getDeviceID() + ", Cancoder ID = " + this.mCanCoder.getDeviceID() + "]";
	}

}
