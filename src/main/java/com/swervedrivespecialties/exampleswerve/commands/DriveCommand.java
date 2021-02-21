package com.swervedrivespecialties.exampleswerve.commands;

import frc.robot.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {
    private double rotation, slow;
    private boolean feild = false;
    public DriveCommand(){
      requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
      if (Robot.getOi().getPrimaryJoystick().getRawButton(7))
        feild = !feild;
      else if (Robot.getOi().getPrimaryJoystick().getPOV() == -1)
      {}
      if (RobotMap.primaryJoystick.getRawButton(6))
        slow = 0.2;
      else
        slow = 1;
      double forward = -Robot.getOi().getPrimaryJoystick().getRawAxis(1);
      forward = Utilities.deadband(forward);
      // Square the forward stick
      forward = Math.copySign(Math.pow(forward, 2.0), forward) * slow;

      double strafe = -Robot.getOi().getPrimaryJoystick().getRawAxis(0);
      strafe = Utilities.deadband(strafe);
      // Square the strafe stick
      strafe = Math.copySign(Math.pow(strafe, 2.0), strafe) * slow;
      if (RobotMap.primaryJoystick.getRawButton(2)){
        rotation = subSystem.getInstance().autoAllignment();
      }else {
          //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        rotation = -RobotMap.primaryJoystick.getRawAxis(4);
        rotation = Utilities.deadband(rotation);
          // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
      }
      DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, feild);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
}