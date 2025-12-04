package frc.robot.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

import com.revrobotics.spark.SparkLowLevel;


public class Motor {
    private SparkMax motor;
    private RelativeEncoder encoder;

    private double distance_toggle = 0; // set distance where speed changes

    private double objective_position = 0;

    private double speed = 1;
    private double slow_speed = 0.02; // put reasonable slow speed here


    public Motor()
    {
        motor = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    public void updateData() {
        double current_position = encoder.getPosition();
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Objective Position", objective_position);
    }

    public void poll() {;
        double distance_position = Math.abs(objective_position - encoder.getPosition());
        double direction = Math.signum(objective_position - encoder.getPosition());
        if (distance_position < distance_toggle) {
            setMotor( slow_speed * direction );
        } else {
            setMotor( speed * direction );
        }
    }

    public void setMotor( double speed ) {
        motor.set(speed); // TODO: put speed with encoder velocity, not battery voltage
    }

    public void goToPosition( double position ) {
        objective_position = position;
    }
}
