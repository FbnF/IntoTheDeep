package org.firstinspires.ftc.teamcode.drive;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class TelemetryInfo {
        public static void setTelemetryRate(int myRate) {


                // Get and store the existing (default) minimum interval between
                // Telemetry transmissions from Robot Controller to Driver Station.
                int oldRate = telemetry.getMsTransmissionInterval();

                // Set the minimum interval, provided by the Blocks user.
                telemetry.setMsTransmissionInterval(myRate);

                // For confirmation, get and store the updated interval.
                int newRate = telemetry.getMsTransmissionInterval();

                telemetry.addData("TELEMETRY REFRESH RATE in milliseconds", null);
                telemetry.addData("Default/previous rate", oldRate);
                telemetry.addData("Requested rate", myRate);
                telemetry.addData("Confirmed new rate", newRate);
                telemetry.update();         // display info on Driver Station screen

        }   // end of method setTelemetryRate()
}