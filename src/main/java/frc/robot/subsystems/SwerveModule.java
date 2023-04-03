package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.Constants;
import frc.robot.math.Vec2d;

public class SwerveModule {
   	/** The motor controlling the module's movement. */
	public final WPI_TalonFX drivingMotor;
	/** The motor controlling the module's rotation. */
	public final WPI_TalonFX steeringMotor;
	/** The can coder measuring the module's absolute rotaiton. */
	private final WPI_CANCoder canCoder;
	/** The can coder's rotational offset. This value must be manually set through phoenix tuner. */
	public final double canOffset;
	/** The direction of rotation relative to the center of the drivetrain. */
	private final double rotDireciton;
	/** The steering motor rotation measured in radians. */
	private double motorRotation;
    
	/**
	 * @param driveMotor driving motor ID
	 * @param steeringMotor steering motor ID
	 * @param canCoder can coder ID
	 * @param rotationDirection the steering motor's rotational direction, usually perpendicular to the center of the robot
	 * @param canCoderOffset the can coder's rotational offset
	 */
	public SwerveModule(int driveMotorID, int steeringMotorID, int canCoderID, double rotationDirection, double canCoderOffset)
	{
		this.drivingMotor = new WPI_TalonFX(driveMotorID);
		this.steeringMotor = new WPI_TalonFX(steeringMotorID);
		this.canCoder = new WPI_CANCoder(canCoderID);
		this.canOffset = canCoderOffset;
		this.rotDireciton = rotationDirection;
		this.motorRotation = 0;
	
	}

    public void init() {
        		// Reset the steering motor.
		this.steeringMotor.configFactoryDefault();
		this.drivingMotor.configFactoryDefault();
		int DRIVING_PID_ID = this.drivingMotor.getDeviceID();
		int STEERING_PID_ID = this.steeringMotor.getDeviceID();
		// Miscellaneous settings.
		this.steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, STEERING_PID_ID, Constants.MS_DELAY);
		this.steeringMotor.setSensorPhase(true);
		this.steeringMotor.setInverted(false);
		this.steeringMotor.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.steeringMotor.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.steeringMotor.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.steeringMotor.configPeakOutputReverse(-1d, Constants.MS_DELAY);
		this.steeringMotor.configAllowableClosedloopError(0, STEERING_PID_ID, Constants.MS_DELAY);

		// PID tune the steering motor.
		this.steeringMotor.selectProfileSlot(STEERING_PID_ID, 0);
		this.steeringMotor.config_kF(STEERING_PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringMotor.config_kP(STEERING_PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringMotor.config_kI(STEERING_PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringMotor.config_kD(STEERING_PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);

		this.drivingMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, DRIVING_PID_ID, Constants.MS_DELAY);
		this.drivingMotor.setSensorPhase(true);
		this.drivingMotor.setInverted(false);
		this.drivingMotor.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.drivingMotor.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.drivingMotor.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.drivingMotor.configPeakOutputReverse(-1d, Constants.MS_DELAY);
		this.drivingMotor.configAllowableClosedloopError(0, DRIVING_PID_ID, Constants.MS_DELAY);

		// PID tune the steering motor.
		this.drivingMotor.selectProfileSlot(DRIVING_PID_ID, 0);
		this.drivingMotor.config_kF(DRIVING_PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.drivingMotor.config_kP(DRIVING_PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.drivingMotor.config_kI(DRIVING_PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.drivingMotor.config_kD(DRIVING_PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);
    	System.out.println("done config");

    	this.drivingMotor.configMotionAcceleration(512*9);
    	this.drivingMotor.configMotionCruiseVelocity(15*9*2048);
    	this.drivingMotor.configMotionSCurveStrength(5);
    	this.drivingMotor.configNeutralDeadband(0.01);

        this.canCoder.configMagnetOffset(this.canOffset, Constants.MS_DELAY);
    }

    public void reset() {
        this.steeringMotor.setSelectedSensorPosition(this.canCoder.getAbsolutePosition()*Constants.DEG_TO_TICK*Constants.STEERING_GEAR_RATIO);
        this.motorRotation = this.canCoder.getAbsolutePosition() * Constants.RAD_TO_DEG;
    }

    public void zero(){
        this.reset();
        this.steeringMotor.set(ControlMode.Position,0);
        this.drivingMotor.setSelectedSensorPosition(0);
        this.motorRotation = 0d;
    }

    
	/**
	 * The motor rotation function. Simply rotates the steering motor in the direction of 
	 * the passed vector, and sets the driving motor's speed to the length of the vector. 
	 * The steering motor will take the shortest path of rotation by rotating a maximum of 
	 * 180° in either direction. Calculations by Alex Green and Marcus Kauffman.
	 * 
	 * @param vec the vector representing the module's movement
	 * @notdeprecated in favor of {@link #setMotion(Vec2d) setMotion}.
	 * @see <a href="https://www.desmos.com/calculator/dgkniftpn6">Alex's Calculations</a>
	 */

	public void setAngle(Vec2d vec)
	{
		// Get the angle of the passed vector.
		double angle = vec.getAngle();

		// Only rotate if the angle isn't NaN.
		if(!Double.isNaN(angle))
		{
			// The steering motor's rotation bounded between 0 and 2π.
			double motorAngle = this.motorRotation % Constants.TWO_PI;
			// The distance between the motor's current rotation and the passed rotation.
			double angleDif = motorAngle - angle;
			// An overcomplicated way of getting the shortest distance to to the passed vector.
			this.motorRotation -= angleDif + (Math.abs(angleDif) > Math.PI ? (motorAngle > angle ? -Constants.TWO_PI : Constants.TWO_PI) : 0d) + Constants.PI_OVER_TWO;
		}

		// Update the two motor's with the new values.
 		this.steeringMotor.set(ControlMode.Position, (this.motorRotation) * Constants.RAD_TO_TICK * 12.8d);
	}

    /// sets the speed and angle of the module.
    public void setMotion(Vec2d vec) {
        this.setAngle(vec);
        this.drivingMotor.set(ControlMode.PercentOutput, vec.getLength());
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
		return this.drivingMotor;
	}

	/**
	 * Getter for the steering motor.
	 * 
	 * @return The steering motor.
	 */
	public WPI_TalonFX getSteeringMotor()
	{
		return this.steeringMotor;
	}

	/**
	 * Getter for the can coder.
	 * 
	 * @return The can coder
	 */
	public WPI_CANCoder getCanCoder()
	{
		return this.canCoder;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.steeringMotor.getDeviceID() + ", Driving Motor ID = " + this.drivingMotor.getDeviceID() + ", Cancoder ID = " + this.canCoder.getDeviceID() + "]";
	}
}