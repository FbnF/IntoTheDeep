package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmControl {


    // Arm control variables
    private DcMotorEx ArmMotor;
    private double initialPosDeg=-20;
    private double DepositAngle=83;
    private double IntakeAngle= -10;

    private double RuntoPositionPower=0.45;


    private int desArmPosTick;
    private double degreesPerTick = 360.0 / 5/1425.1;

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
        ArmMotor.setPower(0.7);
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

}