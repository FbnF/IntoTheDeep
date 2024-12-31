package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ObserverSide_Auto extends LinearOpMode {

    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);
        armControl.setDesArmPosDeg(-5);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);

        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        //Gripper closed state
        gripper.setGripperClosed();
        //Gripper holder to the side
        gripper.setGripperHolderParallel();
        gripper.setAnglerSide();


        // Define starting position
        Pose2d startPos = new Pose2d(8, 53, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(36, 63, Math.toRadians(0));
        Pose2d SamplePickUpPos1 = new Pose2d(29.3, 20, Math.toRadians(0));
        Pose2d SpecimenDropoffPos2 = new Pose2d(32, 58.5, Math.toRadians(0));
        Pose2d SamplePickUpPos2 = new Pose2d(28.5, 12, Math.toRadians(0));
        Pose2d SampleDropoffPos = new Pose2d(25, 28, -Math.toRadians(135));
        Pose2d Specimen2WaitPos = new Pose2d(18, 44, -Math.toRadians(90));
        Pose2d SpecimenPickupPos = new Pose2d(17.5, 27, -Math.toRadians(90));
        Pose2d ParkPos = new Pose2d(10, 21, Math.toRadians(0));

        // Define the trajectory sequence for the Observer side
        TrajectorySequence StageRedObserver = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set the gripper and arm in the right position for Specimen drop off
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(74);})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setAnglerForward();})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setGripperHolderPerpendicular();})
                .waitSeconds(0.5)

                // Step 2: Move the robot to the Specimen drop off position and move forward,
                // then set the Arm down to prepare for placing the Specimen
                .lineToLinearHeading(SpecimenDropoffPos)
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(40);})
                .waitSeconds(0.2)

                // Step 3: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmPower(0))


                // Step 4: Move to Sample 1 and extend the slide to pick up the Sample 1
                .lineToLinearHeading(SamplePickUpPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> gripper.setGripperClosed())
                .UNSTABLE_addTemporalMarkerOffset(1.7, () -> sliderControl.setDesSliderLen(2))
                .waitSeconds(2)

                //Step 5: Turn right 135 degree to drop off Sample 1
                .lineToLinearHeading(SampleDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> sliderControl.setDesSliderLen(0))
                .waitSeconds(1)

                // Step 6: Go to the wait position for picking up Specimen 2
                .lineToLinearHeading(Specimen2WaitPos)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> gripper.setGripperClosed())
                .waitSeconds(1.5)
                /*
                // Step 6: Turn back and strafe right 12 inch to pick up Sample 2
                .lineToLinearHeading(SamplePickUpPos2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperClosed())
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> sliderControl.setDesSliderLen(0))
                .waitSeconds(1)

                // Step 7: Goto Sample drop off position  to drop off Sample 2
                .lineToLinearHeading(SampleDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> gripper.setGripperOpen())
                .waitSeconds(1.5)

                 */
                // Step 8: Pick up Specimen 2

                //.UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                //.lineToLinearHeading(SpecimenPickupPos)
                // Step 9: Turn left 135 degrees and raise the Arm to prepare for
                // Specimen 2 drop off attempt
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(74);})
                .turn(Math.toRadians(135))

                // Step 10: Specimen 2 drop off attempt
                .lineToLinearHeading(SpecimenDropoffPos2)
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {armControl.setDesArmPosDeg(40);})
                .waitSeconds(0.2)
                // Step 11: Move backward and open the Gripper to place and release the Specimen.
                // At the same time, drop the arm all the way down and set its power to zero
                // afterwards
                .back(7.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmPower(0))

                // Step 12: Strafe right to park
                .lineToLinearHeading(ParkPos)
                .waitSeconds(2)

                // Final build for this trajectory
                .build();

        // Wait for start signal
        waitForStart();

        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedObserver);



    }
}