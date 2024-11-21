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



    //Constants for gripper
    //larer numbers are more clockwise


    private static final double Gripper_STOP = 0.5; //speed servo stopped
    private static final double Gripper_FORWARD = 1.0; //full speed forward
    private static final double Gripper_REVERSE = 0.0; //full speed backward


    private static final double      ANGLER_UP     = 1.0; // deposit the pixel
    private static final double      ANGLER_DOWN      = 0.65; // Loading the pixel
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
        gripper = hwMap.get(Servo.class,"gripper"); //Exp Hub port 4
        angler = hwMap.get(Servo.class,"angler"); // Exp Hub port 0
        gripper.setDirection(Servo.Direction.REVERSE);


    }

    public void gripperReverse(double factor){
        //makes new level by taking factor to adjust the right position
        double newLevel = 0.5+factor*0.5;
        //finding reverse control, between 0-0.5
        newLevel= Math.min(0.5, Math.max(Gripper_REVERSE,newLevel));
        //adjusts
        gripper.setPosition(newLevel);

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

}
