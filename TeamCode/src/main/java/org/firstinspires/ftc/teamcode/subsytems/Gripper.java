package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import static java.lang.Thread.sleep;

@Config // this is so the dashboard will pick up variables
public class Gripper {

    //Define Hardware Objects
    private Servo gripper     = null;
    private Servo angler      = null;
    private Servo gripperholder = null;



    // Gripper subsystem constants
    // GripperHolderInit: The initial position for the gripperholder servo
    private double GripperHolderInit=0.42;
    //GripperHolderRotPos: The position after rotation for the gripperholder servo
    private double GripperHolderRotPos=0.76;

    //GripperOpen: The Gripper in the open position to prepare for pick up
    // sample/specimen for gripper servo
    private double GripperOpen=0.42; //0.42
    //GripperClose: The Gripper in the close position for pick up sample/specimen
    // for gripper servo
    private double GripperClose=0.21;//0.25

    //AnglerInit: The initial position for the angler servo
    private double AnglerInit=0.30;
    //AnglerRotPos: The Position after Rotation for the angler servo
    private double AnglerRotPos=0.66;
    //larer numbers are more clockwise


    private static final double      ANGLER_UP     = 0.4; //1.0 parallel to ground
    private static final double      ANGLER_DOWN      = 0.1; //0.67 parallel to the side
    Telemetry       telemetry;
    LinearOpMode    opmode; // need content from Linear opmodes here. Elapsed time mainly

    ElapsedTime runtime = new ElapsedTime();


    public double  targetHeight;
    public double  targetAngle;
    public boolean turnerDown = true;
    public boolean inLevelZero = false;

    /// constructor with opmode passed in
    public Gripper(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void init(HardwareMap hwMap)  {
        // Initialize the gripper
        //Control Hub port 0
        gripper = hwMap.get(Servo.class,"gripper");
        //Control Hub port 2
        gripperholder = hwMap.get(Servo.class,"gripperholder");
        //Control Hub port 1
        angler = hwMap.get(Servo.class,"angler");
    }

    // Gripper open function
    public void setGripperOpen() {
        //Gripper Open state
        setGripperPosition(GripperOpen);
    }
    // Return Gripper close position function
    public double getGripperClosePos() {
        //Gripper close position
        return GripperClose;
    }
    // Return Gripper open position function
    public double getGripperOpenPos() {
        //Gripper open position
        return GripperOpen;
    }
    // Return Gripper current position function
    public double getGripperCurPos() {
        //Gripper close position
        return gripper.getPosition();
    }

    // Gripper open position function
    public void setGripperClosed() {
        //Gripper Close state
        setGripperPosition(GripperClose);
    }

    // Gripper Holder parallel to the holding bar
    public void setGripperHolderParallel() {
        setGripperHolderPosition(GripperHolderInit);
    }
    // Gripper Holder perpendicular to the holding bar
    public void setGripperHolderPerpendicular() {

        setGripperHolderPosition(GripperHolderRotPos);
    }

    // Gripper system facing the side
    public void setAnglerSide() {
        //Gripper system Side Position
        setAnglerPosition(AnglerInit);
    }
    // Gripper system facing forward
    public void setAnglerForward() {
        //Gripper system Forward Position
        setAnglerPosition(AnglerRotPos);
    }

    public void setAnglerPosition(double pos_request) {
        angler.setPosition(pos_request);}
    public void setGripperPosition(double pos_request) {
        gripper.setPosition(pos_request);}
    public void setGripperHolderPosition(double pos_request) {
        gripperholder.setPosition(pos_request);}


}
