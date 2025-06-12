package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;

public class ElevatorSubsystem extends SubsystemBase {
    // Rotations per meter refers to how many full rotations of the motor correlate to one meter up by the elevator
    private static final double kRotationsPerMeter = 10.0;

    // TalonFX is the name of the motor controllers we use (essentially the motor in code)
    private final TalonFX motor;  
    private double targetHeight;
    private final PositionVoltage positionRequest;
    
    public ElevatorSubsystem() {
        // The number within the parentheses refers to the ID we assign to the motor (so we can differentiate between motors)
        motor = new TalonFX(1);

        // If you are unfamiliar with PID control, don't worry about the next 5 lines of code
        // At an extremely basic level, these numbers & applied configuration give us some values that help us control the mechanism
        // You can copy these numbers and lines to your arm code
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 2.0;
        slot0.kI = 0.0;
        slot0.kD = 0.1;
        motor.getConfigurator().apply(slot0);

        // We will get into what this does but for now, just know that this is an object that is reused later in setHeight to set the position of the motor
        positionRequest = new PositionVoltage(0).withSlot(0);
    }

    public void setHeight(double heightMeters) {
        targetHeight = heightMeters;
        motor.setControl(positionRequest.withPosition(heightMeters * kRotationsPerMeter));
    }

    public double getHeight() {
        return motor.getRotorPosition().getValue() / kRotationsPerMeter;
    }

    // Sets the encoder to read 0
    public void zeroEncoder() {
        motor.getRotorPosition().set(0.0);
    }

    public boolean isAtTargetHeight(double toleranceMeters) {
        return Math.abs(getHeight() - targetHeight) <= toleranceMeters;
    }

    public void stop() {
        setHeight(getHeight());
    }
}
