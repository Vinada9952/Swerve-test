package frc.robot.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;

import com.revrobotics.spark.SparkLowLevel;


public class Motor {
    private SparkMax motor;
    private RelativeEncoder encoder;

    private double objective_position = 0;

    private double speed = 0;

    public Motor()
    {
        motor = new SparkMax(8, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    public void updateData() {
        double current_position = encoder.getPosition();
        speed = Math.abs((objective_position-current_position)/20);
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Objective Position", objective_position);
        SmartDashboard.putNumber("Speed", speed);
    }

    public void poll() {
        double current_position = encoder.getPosition();
        if( current_position < objective_position ) {
            setMotor(speed);
        }
        else if( current_position > objective_position ) {
            setMotor(speed*-1);
        }
        else {
            setMotor(0);
        }
    }

    public void setMotor( double speed ) {
        motor.set(speed);
    }

    public void goToPosition( double position ) {
        objective_position = position;
    }
}
