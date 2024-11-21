package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmControl {

    // PID and Feedforward Coefficients
    public static double kP = 0.5; // Proportional Gain,
    public static double kI = 0.9; // Integral Gain
    public static double kD = 0.0; // Derivative Gain
    public static double kS = 0.1; // Static friction compensation
    public static double kG = 5; // Gravity compensation

    // Sample Time (in seconds) for the PID loop
    public double Ts = 0.01;

    // Battery voltage (adjust based on actual readings or keep static)
    public double batteryVoltage = 12.0;

    // Arm control variables
    private DcMotorEx armMotor;
    private double actArmPosDeg = 0.0;
    private double initialPosDeg=-22;

    private double errPrev = 0.0;
    private double errAccum = 0.0;

    private double desArmPosDeg = 80.0;
    private double degreesPerTick = 360.0 / 5/1425.1;
    private int initstep=1;


    // Constructor to initialize the arm motor
    public ArmControl(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public double getActArmPosDeg(){
        return actArmPosDeg;
    }
    // Method to set the target position in degrees
    public void setDesArmPosDeg(double desArmPosDeg) {
        this.desArmPosDeg = desArmPosDeg;
    }

    // Update method for PIDF control
    //public void update(double kP, double kI, double kD, double kS, double kG) {
    public void update() {
        // Read the encoder and convert ticks to degrees
        int ticks = armMotor.getCurrentPosition();

        this.actArmPosDeg = ticks * degreesPerTick + initialPosDeg;

        // Calculate error
        double err = this.desArmPosDeg - this.actArmPosDeg;

        // Feedforward calculations
        double staticFrictionCompensation = Math.signum(err) * kS;
        double gravityCompensation = Math.cos(Math.toRadians(this.actArmPosDeg)) * kG;
        double feedForwardVoltage = staticFrictionCompensation + gravityCompensation;

        // PID calculations
        this.errAccum += err * Ts;
        double pTerm = err * kP;
        double iTerm = this.errAccum * kI;
        double dTerm = ((err - this.errPrev) / Ts) * kD;
        this.errPrev = err;
        double feedbackVoltage = pTerm + iTerm + dTerm;

        // Calculate motor command as a fraction of available battery voltage
        double motorCmd = (feedbackVoltage + feedForwardVoltage) / batteryVoltage;
        motorCmd = Math.max(-1.0, Math.min(1.0, motorCmd)); // Limit to motor power range [-1, 1]

        // Apply power to the motor
        armMotor.setPower(motorCmd);
    }
    public void latch() {
        // Read the encoder and convert ticks to degrees
        //int ticks = ;
        if (initstep==1) {
            this.desArmPosDeg = armMotor.getCurrentPosition() * degreesPerTick + initialPosDeg;
            ; // Adjust this based on your encoder specifications
            initstep=0;
        }
        this.actArmPosDeg = armMotor.getCurrentPosition()*degreesPerTick + initialPosDeg;

        // Calculate error
        double err = this.desArmPosDeg - this.actArmPosDeg;

        // Feedforward calculations
        double staticFrictionCompensation = Math.signum(err) * kS;
        double gravityCompensation = Math.cos(Math.toRadians(this.actArmPosDeg)) * kG;
        double feedForwardVoltage = staticFrictionCompensation + gravityCompensation;

        // PID calculations
        this.errAccum += err * Ts;
        double pTerm = err * kP;
        double iTerm = this.errAccum * kI;
        double dTerm = ((err - this.errPrev) / Ts) * kD;
        this.errPrev = err;
        double feedbackVoltage = pTerm + iTerm + dTerm;

        // Calculate motor command as a fraction of available battery voltage
        double motorCmd = (feedbackVoltage + feedForwardVoltage) / batteryVoltage;
        motorCmd = Math.max(-1.0, Math.min(1.0, motorCmd)); // Limit to motor power range [-1, 1]

        // Apply power to the motor
        armMotor.setPower(motorCmd);
    }
}