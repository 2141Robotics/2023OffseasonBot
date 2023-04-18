package frc.robot.quail;


public class SwerveOdometry {
    public final SwerveDrive swerveDrive;
    public double x;
    public double y;
    public double rotation;
    

    public SwerveOdometry(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }
}
