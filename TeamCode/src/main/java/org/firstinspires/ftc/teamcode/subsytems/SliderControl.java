package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SliderControl {
    // Arm control variables
    private DcMotorEx SliderMotor;
    
    //Pitch Diameter in mm
    private double PitchDiameter=38.2;
    
    // GoBilda 312rpm motor for two stage slider
    private double Inch_Per_Tick = PitchDiameter*Math.PI/25.4/537.7;
    // Intake slider length in Inch
    private double DepositLength= 17.5;
    
    // Retract length in Inch
    private double IntakeLength= 5;

    private double RuntoPositionPower=0.45;


    private int desSliderPosTick;
    LinearOpMode opmode;
    public SliderControl(LinearOpMode opmode) {
        this.opmode = opmode;
    }
    // Constructor to initialize the arm motor
    public void init(HardwareMap hardwareMap) {
        SliderMotor = hardwareMap.get(DcMotorEx.class, "TwoStageMotor");
        SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SliderMotor.setDirection(DcMotor.Direction.REVERSE);
        SliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // getSliderLen: Return the current slider length in Inch
    public double getSliderLen(){
        return SliderMotor.getCurrentPosition() * Inch_Per_Tick;
    }

    // getTgtSliderTick: Return the Target slider length in Ticks

    public int getTgtSliderTick(double TgtLen){
        return (int)(TgtLen/Inch_Per_Tick);
    }

    // getActSliderTick: Return the Current slider length in Ticks
    public int getActSliderTick(){
        return SliderMotor.getCurrentPosition();
    }

    // getTgtSliderTick: Return the Target slider length in Ticks
    public int getTgtSliderTick(){
        return SliderMotor.getTargetPosition();
    }

    // Run to position for the desired Slider Length in Inch
    public void setDesSliderLen(double tgtSliderLen) {
        desSliderPosTick = (int)(tgtSliderLen/Inch_Per_Tick);

        SliderMotor.setTargetPosition(desSliderPosTick);
        SliderMotor.setPower(RuntoPositionPower);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //setSliderIntake: Set slider length to IntakeLength
    public void setSliderIntake() {
        setDesSliderLen(IntakeLength);
    }

    //setSliderRetract: Set slider length to be 2in less than current length
    public void setSliderRetract(double CurLen, double RetractLen) {
        setDesSliderLen(CurLen-RetractLen);
    }

    //setSliderIntake: Set slider length to the Deposit length for deposit
    public void setSliderDeposit() {
        setDesSliderLen(DepositLength);
    }


    // Wrapper method to set the SliderMotor power level to PwrLevel
    public void setSliderPower(double PwrLevel){
        SliderMotor.setPower(PwrLevel);
    }

    // Wrapper method to get the SliderMotor power level
    public double getSliderPower(){
        return SliderMotor.getPower();
    }

    // Wrapper method to set the SliderMotor run mode to using encoder
    public void SliderRunModReset(){
        SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SliderMotor.setPower(0);
    }
}
