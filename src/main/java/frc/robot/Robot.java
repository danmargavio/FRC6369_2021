package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.ArrayList;

public class Robot extends TimedRobot {
    public static double TX, TY;
    public double positions[];
    public double waypoint_1[] = {274.4, 224};
    public double waypoint_2[] = {269.5, 170.1};
    private static OI oi;
    //private static dashboard6369 d;
    public static DigitalInput limit = new DigitalInput(0);
    public static TalonSRX control_panel = new TalonSRX(RobotMap.CONTROL_PANEL);
    // Game Data
    //private String gameData;
    // Camera
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");   
    //CameraServer server;
    //test
    double rotation;
    public double auto_rotation_command, auto_command[];
    //private static DrivetrainSubsystem drivetrain;
    public Timer time;
    public double northPoleAngle = 0;
    public boolean stateFlag, shootState, shooterSpeedState;
    public double offsetTrench = 0;
    public double offset = 0.025;
    public boolean secondAuto = true;
    public boolean reachedFirstPosition = false;
    public static OI getOi() {
        return oi;
    }

    public class waypoint{
        private double x_pos;
        private double y_pos;
        public waypoint(double x, double y) {
            this.x_pos = x;
            this.y_pos = y;
        }
        public double getX() {
            return x_pos;
        }
        public double getY() {
            return y_pos;
        }
    }


    @Override
    public void robotInit() {
        oi = new OI();
         
        ArrayList<waypoint> slalomWaypoints = new ArrayList<waypoint>();       
        slalomWaypoints.add(new waypoint(274.4, 224));
        slalomWaypoints.add(new waypoint(269.5, 170.1));

        DrivetrainSubsystem.getInstance();
        dashboard6369.getInstance();
        subSystem.getInstance().configshooter();
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(320, 240);
        //camera.setFPS(20);
        stateFlag = false;
        shootState = false;
        //VideoSink source = CameraServer.getInstance().addSwitchedCamera("Switched Camera");
        //source.setSource(camera);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // 0 is normal vision , 3 is lights on
    }

    @Override
    public void robotPeriodic() {
        TX = tx.getDouble(0.0);
        Scheduler.getInstance().run();
        // Camera
        TY = ty.getDouble(0.0);
        dashboard6369.tx.setDouble(tx.getDouble(0.0));
        dashboard6369.ty.setDouble(ty.getDouble(0.0));
        dashboard6369.ta.setDouble(ta.getDouble(0.0));
        //dashboard6369.limit.setBoolean(limit.get());
        //if (dashboard6369.Auto.getDouble(0) == 1)
            //secondAuto = true;
        //else
            //secondAuto = false;
        //gameData = DriverStation.getInstance().getGameSpecificMessage();
        //SmartDashboard.putString("Color", gameData);
        SmartDashboard.putNumber("Distance", subSystem.getInstance().distance_estimator(TY));
        SmartDashboard.putString("coordinates", "" + subSystem.getInstance().field_position(new double[]{DrivetrainSubsystem.getInstance().getGyro().getAngle(), subSystem.getInstance().distance_estimator(TY)}));
    }

    public void autonomousInit(){

        // ADD AUTONAV CODE HERE
        // getCompassHeading() // get your compass heading
        // getYaw() // get the angle of the robot relative to the starting point angle

        // northPoleAngle = getCompassHeading()

        // Determine path set based on placement/orientation of robot (discussion needed) OR use playserstation.GameData
        // Select the approproate waypoints for this desired path
        

    }

    public void autonomousPeriodic(){

        // ADD AUTONAV CODE HERE

        // MAKE SURE THE ROBOT KNOWS ITS CURRENT POSITION, ROTATE TOWARDS TOWER AS NEEDED
        if (secondAuto) {
            auto_rotation_command = -(-180 - DrivetrainSubsystem.getInstance().getGyro().getYaw())/500; 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(4);
            DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), auto_rotation_command, false);
            if ((DrivetrainSubsystem.getInstance().getGyro().getYaw() + 180 < 10) && (DrivetrainSubsystem.getInstance().getGyro().getYaw() + 180 > - 10)) {
                secondAuto = false;
            }
        }
        else {
            DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), subSystem.getInstance().autoAllignment(), false);
            positions = subSystem.getInstance().getRobotPosition(DrivetrainSubsystem.getInstance().getGyro().getYaw() + 180, subSystem.getInstance().distance_estimator(ty.getDouble(0.0)));
            if (subSystem.getInstance().areWeThereYet(positions[0], positions[1], waypoint_1[0], waypoint_1[1])) {
                reachedFirstPosition = true;
            }
            else {
                auto_command = subSystem.getInstance().calc_x_and_y_command(positions[0], positions[1], waypoint_1[0], waypoint_1[1]);
                DrivetrainSubsystem.getInstance().drive(new Translation2d(auto_command[0], auto_command[1]), subSystem.getInstance().autoAllignment(), true);
            }
        }
            // If waypoint distance <5ft but >1ft scale speed down relative to distance, bottom out speed at .1
        // DETERMINE IF I HAVE REACHED THE CURRENT WAYPOINT AND IF SO, TRACK WAYPOINT COMPLETION
            // If waypoint reached navigate to next waypoint using above logic

         
             //   DrivetrainSubsystem.getInstance().drive(new Translation2d(0, 0), subSystem.getInstance().autoAllignment(), false);
        
    }

    public void teleopPeriodic(){
       subSystem.getInstance().shoot(95000, false);
       subSystem.getInstance().hopper(0, false);
       subSystem.getInstance().intake(false);
       //subSystem.getInstance().climber();  // - Dan commented out 1/16
       subSystem.getInstance().camera();
       subSystem.getInstance().letMeShoot(0, false);
       dashboard6369.cellShootDetect.setBoolean(subSystem.getInstance().cell_shoot_detect());

       positions = subSystem.getInstance().getRobotPosition(DrivetrainSubsystem.getInstance().getGyro().getYaw() + 90, subSystem.getInstance().distance_estimator(ty.getDouble(0.0)));
       dashboard6369.xpos.setDouble(positions[0]);
       dashboard6369.ypos.setDouble(positions[1]);
       //ControlPanel.colorAlign(gameData);
    }
    
    public void testPeriodic(){
    /*
        if (RobotMap.primaryJoystick.getRawButton(3))
            subSystem.winch.set(ControlMode.PercentOutput, -0.4);
        else if (RobotMap.primaryJoystick.getRawButton(2))
            subSystem.winch.set(ControlMode.PercentOutput, 0.4);
        else
            subSystem.winch.set(ControlMode.PercentOutput, 0);
        if (RobotMap.primaryJoystick.getRawButton(8))
            subSystem.hook_lift.set(ControlMode.PercentOutput, 0.6);
        else if (RobotMap.primaryJoystick.getRawButton(7))
            subSystem.hook_lift.set(ControlMode.PercentOutput, -0.18);
        else
            subSystem.hook_lift.set(ControlMode.PercentOutput, 0);
            */  // - Dan commented out 1/16
    }
}