package frc.robot.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.data.StickPosition;

import java.lang.Math;

import com.revrobotics.spark.SparkLowLevel;


public class DriveFrame {
    Translation2d front_left_motor_position;
    Translation2d front_right_motor_position;
    Translation2d back_left_motor_position;
    Translation2d back_right_motor_position;

    SwerveModuleState front_left_module_state;
    SwerveModuleState front_right_module_state;
    SwerveModuleState back_left_module_state;
    SwerveModuleState back_right_module_state;

    SwerveDriveKinematics kinematic_drive;

    public DriveFrame() {

        front_left_motor_position = new Translation2d(0.5, 0.5); // 0.5 front 0.5 left
        front_right_motor_position = new Translation2d(0.5, -0.5); // 0.5 front 0.5 right
        back_left_motor_position = new Translation2d(-0.5, 0.5); // 0.5 back 0.5 left
        back_right_motor_position = new Translation2d(-0.5, -0.5); // 0.5 back 0.5 right

        ChassisSpeeds speeds;

        kinematic_drive = new SwerveDriveKinematics(
            front_left_motor_position,
            front_right_motor_position,
            back_left_motor_position,
            back_right_motor_position
        );
    }

    void drive( StickPosition left_stick, StickPosition right_stick ) {
        double x_speed = left_stick.y(); // Forward/backward
        double y_speed = left_stick.x(); // Left/right
        double rot_speed = right_stick.x(); // Rotation

        ChassisSpeeds speeds = new ChassisSpeeds(x_speed, y_speed, rot_speed);
        // TODO: display speeds on dashboard
        SwerveModuleState[] module_states = kinematic_drive.toSwerveModuleStates(speeds);

        front_left_module_state = module_states[0];
        front_right_module_state = module_states[1];
        back_left_module_state = module_states[2];
        back_right_module_state = module_states[3];
        // TODO: display modules states on dashboard
    }
}
