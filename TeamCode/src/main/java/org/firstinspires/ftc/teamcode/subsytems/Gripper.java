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
    private double GripperHolderInit=0.52;
    //GripperHolderRotPos: The postion after rotation for the gripperholder servo
    private double GripperHolderRotPos=0.87;

    //GripperOpen: The Gripper in the open position to prepare for pick up
    // sample/specimen for gripper servo
    private double GripperOpen=0.93;
    //GripperOpen: The Gripper in the close position for pick up sample/specimen
    // for gripper servo
    private double GripperClose=0.80;

    //AnglerInit: The initial position for the angler servo
    private double AnglerInit=0.0;
    //AnglerRotPos: The Position after Rotation for the angler servo
    private double AnglerRotPos=0.3;
    //larer numbers are more clockwise


    private static final double Gripper_STOP = 0.5; //speed servo stopped
    private static final double Gripper_FORWARD = 1.0; //full speed forward
    private static final double Gripper_REVERSE = 0.0; //full speed backward


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

    //* The following should all be removed once the update is complete

    public void gripperReverse(double factor){
        //makes new level by taking factor to adjust the right position
        double newLevel = 0.5+factor*0.5;
        //finding reverse control, between 0-0.5
        newLevel= Math.min(0.5, Math.max(Gripper_REVERSE,newLevel));
        //adjusts
        gripper.setPosition(newLevel);

    }
    // Gripper open function
    public void setGripperOpen() {
        //Gripper open state
        setGripperPosition(GripperOpen);
    }
    // Gripper close function
    public void setGripperClosed() {
        //Gripper open state
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

    public void gripperForward(double factor){
        //new level factor to adjust right position
        double newLevel = factor*0.5+0.5;
        //forward, between 0.5-1
        newLevel = Math.min(Math.max(0.5,newLevel),Gripper_FORWARD);
        //adjusts
        gripper.setPosition(newLevel);
    }
    public void gripperStopped(){
        gripper.setPosition(Gripper_STOP);
    }

    public void setAnglerUP() {
        angler.setPosition(ANGLER_UP);//fwd
    }
    public void setAnglerDown() {
        angler.setPosition(ANGLER_DOWN);//fwd
    }
    public void setAnglerInit() {
        angler.setPosition(0.0);}

    //*

    public void setAnglerPosition(double pos_request) {
        angler.setPosition(pos_request);}
    public void setGripperPosition(double pos_request) {
        gripper.setPosition(pos_request);}
    public void setGripperHolderPosition(double pos_request) {
        gripperholder.setPosition(pos_request);}


}
