package frc.robot;
import java.lang.Math;
public class Constants {
    public static int MS_DELAY = 50;

    public static final double[] PID_SETTINGS =  { 0d, 0.5d, 0.0d, 0.0d };

	/** 2π */
	public static final double TWO_PI = 2d * Math.PI;

	/** π/2 */
	public static final double PI_OVER_TWO = Math.PI / 2d;

	/** Number of inches each drive encoder tick is. */
	public static final double INCHES_PER_TICK = Math.PI / 3456d;

	/** Number of ticks in one inch. */
	public static final double TICKS_PER_INCH = 3456d / Math.PI;

	/** Multiply a degree value by this to convert it to radians. */
	public static final double DEG_TO_RAD = Math.PI / 180d;

	/** Multiply a degree value by this to convert it to encoder ticks. */
	public static final double DEG_TO_TICK = 2048d / 360d;

	/** Multiply a radian value by this to convert it to degrees. */
	public static final double RAD_TO_DEG = 180d / Math.PI;

	/** Multiply a radian value by this to convert it to encoder ticks. */
	public static final double RAD_TO_TICK = 2048d / Constants.TWO_PI;

	/** Multiply an encoder tick value by this to convert it to degrees. */
	public static final double TICK_TO_DEG = 360d / 2048d;

	/** Multiply an encoder tick value by this to convert it to radians. */
	public static final double TICK_TO_RAD = 2d * Math.PI / 2048d;

	/** Angle precision for rotation */
	public static final double ANGLE_PRECISION = Math.PI / 32;

	/** distance precision of auton */
	public static final double DIST_PRECISION = 1*TICKS_PER_INCH;

	public static final int TICKS_PER_100MS = 2151;//7

	public static final int INCHES_PER_FOOT = 12;

    public static final double STEERING_GEAR_RATIO = 12.8;
}

