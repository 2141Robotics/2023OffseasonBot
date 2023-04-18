package frc.robot.quail;

import java.util.List;
import com.kauailabs.navx.frc.AHRS;
import java.util.Collections;
import java.util.Arrays;

public class _SwerveDrive {
    /** The gryoscope used for rotaiton measurements. */
    private final AHRS gyro;
    /** A list of all of the swerve modules on the drivetrain. */
    private final List<SwerveModule> modules;
    /** The minimum movement speed of the drivetrain. */
    private final double minMove;
    /** The maximum movement speed of the drivetrain. */
    private final double maxMove;
    /** The maximum rotational speed of the robot. */
    private final double maxRot;

    /**
     * @param minSpeed      minimum movement speed (0 to 1)
     * @param maxSpeed      maximum movement speed (0 to 1)
     * @param maxRotation   maximum rotational speed (0 to 1)
     * @param gyroscope     the swerve drive's gyroscope
     * @param swerveModules the swerve drive's wheel modules
     */
    public _SwerveDrive(double minSpeed, double maxSpeed, double maxRotation, AHRS gyroscope,
            SwerveModule... swerveModules) {
        this.minMove = minSpeed;
        this.maxMove = maxSpeed;
        this.maxRot = maxRotation;
        this.gyro = gyroscope;
        this.modules = Collections.unmodifiableList(Arrays.asList(swerveModules));
    }

    /**
     * Move's the swerve drive. The direction vector is automatically adjusted by
     * the
     * gyroscope. Both the magnitude of the directional vector and the rotational
     * speed
     * added together must be less than or equal to 1.
     * 
     * @param directionVec    the direction and speed to travel (magnitude from -1
     *                        to 1)
     * @param rotationalSpeed how fast the robot rotates (from -1 to 1)
     */
    public void move(Vec2d directionVec, double rotationalSpeed) {
        // Vector representation of the speed and direction of the drivetrain.
        Vec2d driveVec = directionVec.rotate(-this.gyro.getYaw() - 90d, true);

        this.modules.forEach(m -> {
            // Vector representation of the modules rotation.
            Vec2d rotVec = new Vec2d(m.getRotationDirection() + Constants.PI_OVER_TWO, rotationalSpeed, false);
            // Add the movement and rotation vectors together.
            Vec2d finalVec = driveVec.add(rotVec);
            // Set the modules movement to the combined vector.
            m.setMotion(finalVec);
        });
    }

	/**
	 * Reset the gyro to 0°.
	 */
	public void resetGyro()
	{
		this.gyro.reset();
	}
	
	/**
	 * Getter for the gyroscope.
	 * 
	 * @return The gyroscope.
	 */
	public AHRS getGyro()
	{
		return this.gyro;
	}

	/**
	 * Getter for the swerve modules.
	 * 
	 * @return The swerve modules.
	 */
	public List<SwerveModule> getModules()
	{
		return this.modules;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {}, ...]"
		StringBuilder builder = new StringBuilder("SwerveDrive[");

		for(int i = 0; i < this.modules.size(); i++)
		{
			builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
		}

		builder.delete(builder.length() - 2, builder.length());
		builder.append("]");
		return builder.toString();
	}

	/** Move for Autonomious */
	public void moveDistance(Vec2d vec) {
		for(SwerveModule module : this.modules) {
			module.setAngle(vec);
			module.drivingMotor.set(0.1);
			//module.drivingMotor.set(ControlMode.MotionMagic, vec.getLength());
		}
	}

	public void stop(){
		for(SwerveModule module : this.modules) {
			module.drivingMotor.set(0);
		}
	}
    
	/**
	 * Average encoder distance of the drive modules
	 * @return distance in encoder ticks before gear ratio
	 */
	public double averageDist() {
		int totaldist = 0;
		for (int i = 0; i <4; i++) {
			totaldist += this.modules.get(i).drivingMotor.getSelectedSensorPosition();
		}
		return totaldist/4;
	}
	public void resetDistance(){
		for (SwerveModule module : this.modules) {
			module.drivingMotor.setSelectedSensorPosition(0d);
		}
	}
}
