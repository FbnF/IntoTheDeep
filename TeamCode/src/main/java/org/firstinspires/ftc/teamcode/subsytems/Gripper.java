package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

@Config // this is so the dashboard will pick up variables
public class Gripper {

    //Define Hardware Objects
    private Servo gripper     = null;
    private Servo angler      = null;
    private Servo gripperholder = null;



    // Gripper subsystem constants
    // GripperHolderParallel: The initial position for the gripperholder servo
    private double GripperHolderParallel=0.42;//0.33;
    //GripperHolderPerpendicular: The position after rotation for the gripperholder servo
    private double GripperHolderPerpendicular=0.77;//0.68;

    //GripperOpen: The Gripper in the open position to prepare for pick up
    // sample/specimen for gripper servo
    private double GripperOpen=0.56;//0.64; //0.42
    //GripperClose: The Gripper in the close position for pick up sample/specimen
    // for gripper servo
    private double GripperClose=0.35;//0.32;//0.25

    //AnglerSamplePos: The initial position for the angler servo to ensure the gripper could
    //pick up the specimen
    private double AnglerSamplePos =0.70;
    //AnglerSpecimenPos: The angler servo Position for the angler servo to allow regular specimen pick up
    private double AnglerSpecimenPos =0.34;
    // AnglerInitPos: Initial angler position to meet the dimension limitation
    private double AnglerInitPos=0.07;
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
        //gripperholder.setDirection(Servo.Direction.REVERSE);
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
        setGripperHolderPosition(GripperHolderParallel);
    }
    // Gripper Holder perpendicular to the holding bar
    public void setGripperHolderPerpendicular() {

        setGripperHolderPosition(GripperHolderPerpendicular);
    }

    // Gripper system facing the side
    public void setAnglerSample() {
        //Gripper system Side Position
        setAnglerPosition(AnglerSamplePos);
    }

    // Gripper system down position for regular pick up
    public void setAnglerSpecimen() {
        //Gripper system Forward Position
        setAnglerPosition(AnglerSpecimenPos);
    }
    // Gripper system init status
    public void setAnglerInit() {
        //Gripper system Forward Position
        setAnglerPosition(AnglerInitPos);
    }

    public void setAnglerPosition(double pos_request) {
        angler.setPosition(pos_request);}
    public void setGripperPosition(double pos_request) {
        gripper.setPosition(pos_request);}
    public void setGripperHolderPosition(double pos_request) {
        gripperholder.setPosition(pos_request);}


}
