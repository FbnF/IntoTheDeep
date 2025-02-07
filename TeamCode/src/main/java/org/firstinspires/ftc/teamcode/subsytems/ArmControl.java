package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmControl {

    // Arm control variables
    private DcMotorEx ArmMotor;
    private Servo hangservo     = null;
    private double initialPosDeg=-20;
    private double DepositAngle=89;
    private double IntakeAngle= -10;

    private double RuntoPositionPower=0.6;

    private int desArmPosTick;
    private double degreesPerTick = 360.0 / 5/1425.1;
    private double HangServoUp= 0.34;
    private double HangServoSide=0.75;

    LinearOpMode    opmode;
    public ArmControl(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    // Constructor to initialize the arm motor
    public void init(HardwareMap hardwareMap) {
        ArmMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servo for hanging: hangservo

        hangservo = hardwareMap.get(Servo.class,"hangservo");
        hangservo.setDirection(Servo.Direction.REVERSE);

    }
    public double getActArmPosDeg(){
        return ArmMotor.getCurrentPosition() * degreesPerTick + initialPosDeg;
    }

    public int getTgtArmPosTick(double TgtAngle){
        return (int)((TgtAngle-initialPosDeg)/degreesPerTick);
    }

    public int getActArmTick(){
        return ArmMotor.getCurrentPosition();
    }

    public int getTgtArmTick(){
        return ArmMotor.getTargetPosition();
    }

    // Run to position for the desired Arm Angle
    public void setDesArmPosDeg(double tgtArmPosDeg) {
        desArmPosTick = (int)((tgtArmPosDeg-initialPosDeg)/degreesPerTick);

        ArmMotor.setTargetPosition(desArmPosTick);
        ArmMotor.setPower(RuntoPositionPower);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Increase the power level to 0.7 for hanging the robot in the end
    public void setArmHanging(double tgtArmPosDeg) {
        desArmPosTick = (int)((tgtArmPosDeg-initialPosDeg)/degreesPerTick);
        ArmMotor.setTargetPosition(desArmPosTick);
        ArmMotor.setPower(0.9);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Set Arm angle to the Deposit angle for deposit
    public void setArmDeposit() {
        setDesArmPosDeg(DepositAngle);
    }

    // Set Arm angle to the intake angle for intake
    public void setArmIntake() {
        setDesArmPosDeg(IntakeAngle);
    }

    // Wrapper method to set the ArmMotor power level to PwrLevel
    public void setArmPower(double PwrLevel){
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(PwrLevel);
    }

    // Wrapper method to get the ArmMotor power level
    public double getArmPower(){
        return ArmMotor.getPower();
    }

    // Wrapper method to set the ArmMotor run mode to using encoder
    public void ArmRunModReset(){
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setPower(0);
    }

    // Wrapper method to set the run mode to using encoder
    public void ArmRunModEncoder(){
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Wrapper method to reset the SliderMotor encoder
    public void ArmEncoderReset(){
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Hangservo control
    // Hang servo up function
    public void setHangServoUp() {
        //Blocking block up to blocking the arm
        setHangServoPosition(HangServoUp);
    }
    // Hang servo side function
    public void setHangServoSide() {
        //Stop blocking the arm
        setHangServoPosition(HangServoSide);
    }
    // Setting hangservo position
    public void setHangServoPosition(double pos_request) {
        hangservo.setPosition(pos_request);}
}