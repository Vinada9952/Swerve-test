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

    PIDController pid = new PIDController(1.2, 1.2, 0.8);

    private double objective_position = 0;


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
        SmartDashboard.putNumber("Speed", pid.calculate(current_position, objective_position));
    }

    public void poll() {
        double current_position = encoder.getPosition();
        setMotor( pid.calculate(current_position, objective_position) );
    }

    public void setMotor( double speed ) {
        motor.set(speed); // TODO: put speed with encoder velocity, not battery voltage
    }

    public void goToPosition( double position ) {
        objective_position = position;
    }
}
