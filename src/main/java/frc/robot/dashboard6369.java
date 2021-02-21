package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
public class dashboard6369 {
    private static dashboard6369 instance;
    public static ShuffleboardTab tab, dash;
    public static NetworkTableEntry shooterPercent;
    public static NetworkTableEntry shooterVelocity;
    public static NetworkTableEntry cellShootDetect;//Dan added
    public static NetworkTableEntry P;
    public static NetworkTableEntry D;
    public static NetworkTableEntry tx;
    public static NetworkTableEntry ty;
    public static NetworkTableEntry ta;
    public static NetworkTableEntry xpos;
    public static NetworkTableEntry ypos;
    public static NetworkTableEntry limit;//Dan added
    public static NetworkTableEntry shootVoltage;
    public static NetworkTableEntry shootVelocity;
    public static NetworkTableEntry shooterTarget;
    public static NetworkTableEntry shootCurrent;
    public static NetworkTableEntry shoot2Current;
    public static NetworkTableEntry shoot2Voltage;
    public static NetworkTableEntry shootAcceleration;
    public static NetworkTableEntry gyro;
    public static NetworkTableEntry northPole; // Dan added this so we can see the North Pole angle
    public static NetworkTableEntry hopperRate;
    public static NetworkTableEntry intakeRate;
    public static NetworkTableEntry hookRate; // hook reference, possibly related to lift?
    public static NetworkTableEntry winchRate; // Winch reference
    public static NetworkTableEntry develop, offset, lift_power, Auto, dzero, switch_control;

    public dashboard6369(){
        tab = Shuffleboard.getTab("control");
        dash = Shuffleboard.getTab("Shuffleboard");
        shooterPercent = tab.add("shoot_percent", 1).getEntry();
        shooterVelocity = tab.add("shoot_velocity", 75000).getEntry(); //Dan added a default value of 75000 for now
        cellShootDetect = tab.add("cell_shoot_detection", 0).getEntry(); //Dan added
        shooterTarget = tab.add("shooter_speed_check", false).getEntry(); //Dan added for shooter speed indicaton
        P = tab.add("Proportional", .1).getEntry();
        D = tab.add("Derivative", 0).getEntry();
        tx = tab.add("LimelightX", 10000000).getEntry();
        ty = tab.add("LimelightY", 10000000).getEntry();
        ta = tab.add("LimelightTA", 10000000).getEntry();
        xpos = tab.add("X Position", 1).getEntry();
        ypos = tab.add("Y Position", 1).getEntry();
        limit = tab.add("Lift Limit", true).getEntry(); // Lift Limit, possibly problematic with no lift
        gyro = tab.add("Gyroscope", 1).getEntry();
        northPole = tab.add("North Angle", 1).getEntry(); // Dan added north pole info to screen
        hopperRate = tab.add("Hopper Rate", 1).getEntry();
        intakeRate = tab.add("Intake Rate", 1).getEntry();
        hookRate = tab.add("Hook Rate", 1).getEntry(); // Hook reference, related to lift?
        winchRate = tab.add("Winch Rate", 1).getEntry(); // Winch reference
        shootVoltage = tab.add("Shooter Voltage", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        shootVelocity = tab.add("Shooter Velocity", 20000).withWidget(BuiltInWidgets.kGraph).getEntry();
        shootCurrent = tab.add("Shooter Current", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        shoot2Voltage = tab.add("Shooter2 Voltage", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        shoot2Current = tab.add("Shooter2 Current", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        shootAcceleration = tab.add("Shooter Acceleration", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        //develop = tab.add("Developer Mode", 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        offset = tab.add("Offset shooter", 0).getEntry();
        lift_power = tab.add("lift percentage", 0.603).getEntry(); // Lift percentage, possible lift problems
        Auto = tab.add("Autonomous Mode", 0).getEntry();
        dzero = tab.add("Zero Drivetrain", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        switch_control = dash.add("Field/Robot relative", true).withWidget(BuiltInWidgets.kToggleSwitch).withSize(2, 2).getEntry();
    }
    /**
     * This looks for the current instance of a dashboard and if it can't find one it creates a new one. You can use this to call
     * dashboard6369 like it is static.
     * @return dashboard6369 object
     */
    public static dashboard6369 getInstance() {
        if (instance == null) {
            instance = new dashboard6369();
        }

        return instance;
    }
}
