package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.opencv.core.Mat;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class subSystem{ // super-class for driveBase

    public static ButtonDebouncer hopperintake_inDebouncer, hopperintake_outDebouncer, climberDebouncer;
    public static TalonSRX intake, shooter, shooter2, winch;
    public static VictorSPX hook_lift, hopperMotor, feeder;
    public static Servo stop;
    public static DoubleSupplier per;
    private double vel1, time1, increment_lift, increment_winch; // Increment winch and lift referenced, possibly problematic
    private static subSystem instance;
    // obstacle format = (x, y, width, length)
    private static final double[][] obstacle = {{}};
    private static final double field_width = 0;
    private static final double field_height = 0;
    private static final double camera_offsetX = 0;
    private static final double camera_offsetY = 0;
    public static double auto_maxSpeed = 0.1; //units are m/s
    public static double position_check_deadband = 10; //units are inches

    public subSystem(){
        hopperMotor = new VictorSPX(RobotMap.HOPPER);
        feeder = new VictorSPX(RobotMap.CONTROL_PANEL);  // - Dan has not yet commented out 1/16
        intake = new TalonSRX(RobotMap.INTAKE);
        shooter = new TalonSRX(RobotMap.SHOOTER);
        shooter2 = new TalonSRX(RobotMap.SHOOTER2);
        //hook_lift = new VictorSPX(RobotMap.HOOK_LIFT);  // - Dan commented out 1/16
        //winch = new TalonSRX(RobotMap.WINCH);  // - Dan commented out 1/16
        stop = new Servo(0);
        vel1 = shooter.getSelectedSensorVelocity();
        time1 = Timer.getFPGATimestamp();
        increment_lift = 0;
        increment_winch = 0;
    }
    /**
     * This looks for the current instance of a subsystem and if it can't find one it creates a new one. You can use this to call
     * subsystem like it is static.
     * @return subsystem object
     */
    public static subSystem getInstance() {
        if (instance == null) {
            instance = new subSystem();
        }
        return instance;
    }
    /**
     * This runs the hopper. On joystick 2 the controls are the right trigger for in and the left trigger for out.
     * @param autoPercent = what percent to run the hopper in autonomous mode
     * @param auto = is the hopper running in autonomous mode
     */
    public void hopper(double autoPercent, boolean auto){
        if(auto){
            hopperMotor.set(ControlMode.PercentOutput, autoPercent);
        }else{
            if (RobotMap.secondaryJoystick.getRawAxis(3) > 0.08)
                hopperMotor.set(ControlMode.PercentOutput, -RobotMap.secondaryJoystick.getRawAxis(3));
            else if (RobotMap.secondaryJoystick.getRawAxis(2) > 0.08)
                hopperMotor.set(ControlMode.PercentOutput, RobotMap.secondaryJoystick.getRawAxis(2));
            else
                hopperMotor.set(ControlMode.PercentOutput, 0);
        }
    }
    /**
     * This runs the intake. On joystick 1 the controls are the right trigger in and the left trigger out.
     * @param auto = is the intake running in autonomous mode
     */
    public void intake(boolean auto){
        if(auto){
            intake.set(ControlMode.PercentOutput, 0.6);
        }else{
            if (RobotMap.primaryJoystick.getRawAxis(3) > 0.08)
                intake.set(ControlMode.PercentOutput, -0.90*RobotMap.primaryJoystick.getRawAxis(3));
            else if (RobotMap.primaryJoystick.getRawAxis(2) > 0.08)
                intake.set(ControlMode.PercentOutput, 0.90*RobotMap.primaryJoystick.getRawAxis(2));
            else
                intake.set(ControlMode.PercentOutput, 0);
        }
    }
    /**
     * This function allows you to switch between camera modes on the limelight. Up is vision targeting, and down gives the driver the ability
     * to see
     */
    public void camera(){
        if (RobotMap.primaryJoystick.getPOV() == 0) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
        }
        else if (RobotMap.primaryJoystick.getPOV() == 180) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        }
        else if (RobotMap.primaryJoystick.getPOV() == -1) {
            // no change
        }
    }
    /**
     * This function sets up the motors on the shooter to the desired configurations
     */
    public void configshooter(){
        shooter2.set(ControlMode.Follower, RobotMap.SHOOTER);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        shooter.setSelectedSensorPosition(0);
        shooter.setSensorPhase(false);
        shooter.setInverted(true);
        shooter.config_kF(0, 0, 10);
		shooter.config_kP(0, 100, 10);
		shooter.config_kI(0, 0, 10);
        shooter.config_kD(0, 0, 10);
        shooter.configClosedLoopPeakOutput(0, 0.95);
        shooter2.configClosedLoopPeakOutput(0, 0.95);
    }
    /**
     * This function runs the shooter. The Y button controls the shooter on Joystick 1.
     * @param shooter_velocity = velocity to run the shooter at
     * @param auto = is the shooter running in autonomous mode
     */
    public void shoot(double shooter_velocity, boolean auto){
        if (auto){
            shooter.set(ControlMode.Velocity, shooter_velocity);
            if (shooter.getSelectedSensorVelocity() > (shooter_velocity * 0.95)) {
                hopper(0.6, true);
                letMeShoot(0.75, true);
            }else{
                hopper(0, true);
                letMeShoot(0, true);
            }
        }else{    
            if (RobotMap.primaryJoystick.getRawButton(4)){
                shooter.set(ControlMode.Velocity, shooter_velocity);
            }
            else {
                shooter.set(ControlMode.PercentOutput, 0);
            }
            // ======== (3/4/2020) Dan added code to below to indicate that shooter speed has reached acceptable velocity and send to dashboard
            if (shooter.getSelectedSensorVelocity() > (shooter_velocity * 0.95)){
                SmartDashboard.putBoolean("shoot ready", true);
                RobotMap.secondaryJoystick.setRumble(RumbleType.kLeftRumble, 1);
            }else{
                SmartDashboard.putBoolean("shoot ready", false);
                RobotMap.secondaryJoystick.setRumble(RumbleType.kLeftRumble, 0);
            }
        }
        // ======== (3/4/2020) Dan added code above to indicate that shooter speed has reached acceptable velocity and send to dashboard
        dashboard6369.shootVelocity.setDouble(shooter.getSelectedSensorVelocity());
        dashboard6369.tx.setDouble(dashboard6369.shooterPercent.getDouble(1.0));
        dashboard6369.shootCurrent.setDouble(shooter.getSupplyCurrent());
        dashboard6369.shootVoltage.setDouble(shooter.getMotorOutputVoltage());
        dashboard6369.shoot2Voltage.setDouble(shooter2.getMotorOutputVoltage());
        dashboard6369.shoot2Current.setDouble(shooter2.getSupplyCurrent());
        dashboard6369.gyro.setDouble(DrivetrainSubsystem.getInstance().getGyro().getYaw()); // Added gyro yaw output to shuffleboard
        dashboard6369.northPole.setDouble(DrivetrainSubsystem.getInstance().getGyro().getCompassHeading());  // Dan added north pole info to the screen
        dashboard6369.shootAcceleration.setDouble(acceleration());
    }
    /**
     * This calculates the acceleration of the shooter.
     * @return acceleration
     */
    public double acceleration(){
        double acc;
        acc = (shooter.getSelectedSensorVelocity() - vel1) / (Timer.getFPGATimestamp() - time1);
        vel1 = shooter.getSelectedSensorVelocity();
        time1 = Timer.getFPGATimestamp();
        return acc;
    }
    /**
     * This detects when the speed of the shooter drops so you can determine when you have shot a ball.
     * @return boolean
     */
    public boolean cell_shoot_detect(){
        if (acceleration() > 6000)
            return true;
        else
            return false;
    }

    public void letMeShoot(double autoPercent, boolean auto){
        if(auto){
            feeder.set(ControlMode.PercentOutput, autoPercent);
        }else{
            if (RobotMap.secondaryJoystick.getRawButton(5)){
                feeder.set(ControlMode.PercentOutput, 0.9);
            }else if (RobotMap.secondaryJoystick.getRawButton(6)){
                feeder.set(ControlMode.PercentOutput, -0.9);
            }else
                feeder.set(ControlMode.PercentOutput, 0);
        }
    }

    public void climber(){
        if (RobotMap.primaryJoystick.getRawButton(8)){
            increment_lift = dashboard6369.lift_power.getDouble(0.95);
        }else if (RobotMap.primaryJoystick.getRawButton(7)){
            increment_lift = -0.9;
        }else{
            increment_lift = 0;
        }
        if (RobotMap.primaryJoystick.getRawButton(9) && RobotMap.secondaryJoystick.getRawButton(7)){
            if (RobotMap.primaryJoystick.getRawButton(2)){
                increment_winch = 0.6;
            }
        }else{
            increment_winch = 0;
        }
        //winch.set(ControlMode.PercentOutput, increment_winch);  // - Dan commented out 1/16
        //hook_lift.set(ControlMode.PercentOutput, increment_lift);  // - Dan commented out 1/16
    }
    /**
     * This locks on to the center of the targeting area by turning the robot until you minimize the tx value
     * @return percent rotation
     */
    public double autoAllignment(){
        double offset = dashboard6369.offset.getDouble(0);
        double tx_1 = Robot.TX; // make a local copy of the tx value
        double tx_2 = (tx_1/29.8) - offset;  // scale the 2nd tx value by the total range of tx (-29.8 to +29.8 degrees)
        double rotation;
        if (Math.abs(tx_2) > 0.50) {
          rotation = -0.2 * Math.signum(tx_2);
        }
        else if (Math.abs(tx_2) < 0.50  && Math.abs(tx_2) >= 0.20) {
          rotation = -0.1 * Math.signum(tx_2);        
        }  // create a minimum control band so that the rotation command doesnt become too small
        else if (Math.abs(tx_2) < 0.20 && Math.abs(tx_2) >= 0.02) {  // arbitrarily set a tolerable tx error band of 2%
          rotation = -0.02 * Math.signum(tx_2);
        }
        else {
          rotation = 0;
        }
        return rotation;
    }
    /**
     * This function takes the limelight camera ty value in degrees and outputs the estimated distance to the robot camera in inches
     * @param ty_angle
     * @return distance
     */
    public double distance_estimator(double ty_angle){
        return 62/Math.tan(Math.PI * (8.686 + ty_angle) / 180);
    }

    /**
     * This function calculates the speed vector in m/s for both the x and y command
     * @param startPositionX //initial position of x
     * @param startPositionY //initial position of y
     * @param endPositionX //ending position of x
     * @param endPositionY //ending position of y
     * @return auto_command //array, first position is x, second position is y
     */
    public double[] calc_x_and_y_command(double startPositionX, double startPositionY, double endPositionX, double endPositionY){
        double auto_command[] = {0,0};
        auto_command[0] = auto_maxSpeed * (endPositionX - startPositionX)/Math.sqrt(Math.pow(endPositionX - startPositionX,2) + Math.pow(endPositionY - startPositionY, 2));
        auto_command[1] = auto_maxSpeed * (endPositionY - startPositionY)/Math.sqrt(Math.pow(endPositionX - startPositionX,2) + Math.pow(endPositionY - startPositionY, 2));
        return auto_command;
    }
/**
     * This function calculates the speed vector in m/s for both the x and y command
     * @param currentPositionX //where we are currently for x
     * @param currentPositionY //where we are currently for y
     * @param endPositionX //Where we need to be for x
     * @param endPositionY //Where we need to be for y
     * @return auto_command //array, first position is x, second position is y
     */
    public boolean areWeThereYet(double currentPositionX, double currentPositionY, double endPositionX, double endPositionY){
        if (((currentPositionX <= (endPositionX + position_check_deadband)) && (currentPositionX >= (endPositionX - position_check_deadband))) && ((currentPositionY <= (endPositionY + position_check_deadband)) && (currentPositionY >= (endPositionY - position_check_deadband))))
            return true;
        else
            return false;
    }

    /**
     * This function returns the angle in degrees that you add to the offset of the shooter. Velocity of the robot
     * perpendicular to the locally referenced 0 degrees.
     * @param xVel = velocity of the robot
     * @param cellVel = velocity of the ball leaving the shooter
     * @return angle
     */
    public double targetCorrectionAngle(double xVel, double cellVel){
        return Math.atan(xVel / cellVel) * (180 / Math.PI);
    }
   /**
     * This function returns x,y position of the robot in inches relative to the bottom left corner of the field
     * @param theta = angle in degrees from the tower relative to the east side of the field
     * @param distance = in inches between the robot the tower
     * @return array {x,y} position in inches of the robot
     */
    public double[] getRobotPosition(double theta, double distance)
    {
        final int towerPosition = 36; //in inches
        double x_y[] = {0,0};
        x_y[0] = distance * Math.cos( Math.PI/180*theta);
        x_y[1] = 180 - (distance * Math.sin(Math.PI/180*theta)) + towerPosition;
        return x_y;
    }
    /**
     * This function returns the linear speed of the ball after leaving the shooter in meters per second.
     * @param vel = velocity of the shooter
     * @return velocity of the ball
     */
    public double ballVelocity(double vel){
        return (vel * (10 / 4096)) * ((2 * 2.54) / 100);
    }
    /**
     * Thsi function returns the horizontal component of the velocity of the ball coming out of the shooter.
     * @param vel = velocity of the shooter
     * @return velocity of the ball relative to the ground
     */
    public double ballVelocityX(double vel){
        return Math.cos(ballVelocity(vel));
    }
    /**
     * This returns an array of the position the robot is on the field. Units in meters
     * @param robot = array of the angle and distance of the robot relative to the target in radians and centimeters
     * @return field array of your position on the field in centimeters
     */
    public double[] field_position(double[] robot){
        double k = (228.655 * 2.54) / 100;
        double[] field = new double[2];
        field[0] = (k + Math.sin(-robot[0]) * robot[1]) - toCentimeters(161.66);
        field[1] = (Math.cos(robot[0]) * robot[1]) + toCentimeters(314.63);
        return field;
    }
    /**
     * This returns an array of the angle and distance relative to the target based on the position on the field. Units in meters
     * @param field = array of the position on the field in centimeters
     * @return robot array of the angle and distance of the robot relative to the target in radians and centimeters
     */
    public double[] robot_position(double[] field){
        double[] robot = new double[2];
        robot[0] = Math.atan(field[0] / field[1]);
        robot[1] = Math.sqrt(Math.pow(field[0], 2) + Math.pow(field[1], 2));
        return robot;
    }
    /**
     * This function path finds the robot to a certain position on the field
     * @param x1 = desired x position
     * @param y1 = desired y position
     */
    public void find_position(double x1, double y1){
        double xpos, ypos;
        double x = field_position(new double[]{DrivetrainSubsystem.getInstance().getGyro().getAngle(), distance_estimator(Robot.TY)})[0];
        double y = field_position(new double[]{DrivetrainSubsystem.getInstance().getGyro().getAngle(), distance_estimator(Robot.TY)})[1];
        if(x < (x1 * 0.95))
            xpos = (x - x1) / field_width;
        else if(x > (x1 * 1.05))
            xpos = (x - x1) / field_width;
        else
            xpos = 0;
        if(y < (y1 * 0.95))
            ypos = (y - y1) / field_height;
        else if(y > (y1 * 1.05))
            ypos = (y - y1)/ field_height;
        else
            ypos = 0;
        DrivetrainSubsystem.getInstance().drive(new Translation2d(ypos, xpos), autoAllignment(), true);
    }
    /**
     * This function determines if there is an obstacle in the position you are trying to go to
     * @param pos = array of current postition
     * @return if there is an obtacle
     */
    public boolean offlimits(double[] pos){
        for(double[] obs : obstacle){
            for(double i = obs[0]; i < obs[3] + obs[0]; i += 0.1)
                for(double j = obs[1]; j < obs[4] + obs[1]; j += 0.1)
                    if(pos.equals(new double[]{i, j}))
                        return true;
        }
        return false;
    }
    /**
     * This function takes a measurment from inches and converts it to centimeters.
     * @param inches
     * @return centimeters
     */
    public double toCentimeters(double inches){
        return inches * 2.54;
    }
}