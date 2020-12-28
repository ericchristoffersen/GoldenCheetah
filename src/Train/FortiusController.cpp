/*
 * Copyright (c) 2011 Mark Liversedge (liversedge@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRAfNTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "FortiusController.h"
#include "Fortius.h"
#include "RealtimeData.h"

FortiusController::FortiusController(TrainSidebar *parent,  DeviceConfiguration *dc) : RealtimeController(parent, dc)
{
    myFortius = new Fortius (parent);
}


int
FortiusController::start()
{
    return myFortius->start();
}


int
FortiusController::restart()
{
    return myFortius->restart();
}


int
FortiusController::pause()
{
    return myFortius->pause();
}


int
FortiusController::stop()
{
    return myFortius->stop();
}

bool
FortiusController::find()
{
    return myFortius->find(); // needs to find either unconfigured or configured device
}

bool
FortiusController::discover(QString) {return false; } // NOT IMPLEMENTED YET


bool FortiusController::doesPush() { return false; }
bool FortiusController::doesPull() { return true; }
bool FortiusController::doesLoad() { return true; }

/*
 * gets called from the GUI to get updated telemetry.
 * so whilst we are at it we check button status too and
 * act accordingly.
 *
 */
void
FortiusController::getRealtimeData(RealtimeData &rtData)
{
    if(!myFortius->isRunning())
    {
        emit setNotification(tr("Cannot Connect to Fortius"), 2);
        parent->Stop(1);
        return;
    }

    // get latest telemetry
    const auto telemetry = myFortius->getTelemetry();

    //
    // PASS BACK TELEMETRY
    //
    rtData.setWatts(telemetry.PowerWatts);
    rtData.setHr(telemetry.HeartRate);
    rtData.setCadence(telemetry.Cadence);
    rtData.setSpeed(telemetry.SpeedMS * 3.6); // to kph


    // post processing, probably not used
    // since its used to compute power for
    // non-power devices, but we may add other
    // calculations later that might apply
    // means we could calculate power based
    // upon speed even for a Fortius!
    processRealtimeData(rtData);

    //
    // BUTTONS
    //

    // ignore other buttons if calibrating
    if (parent->calibrating) return;


    // ADJUST LOAD
    if (telemetry.Buttons & Fortius::FT_PLUS)   parent->Higher();
    if (telemetry.Buttons & Fortius::FT_MINUS)  parent->Lower();

    // LAP/INTERVAL
    if (telemetry.Buttons & Fortius::FT_ENTER)  parent->newLap();

    // CANCEL
    if (telemetry.Buttons & Fortius::FT_CANCEL) parent->Stop(0);


    // Ensure we set the UI load to the actual setpoint from the fortius (as it will clamp)
    rtData.setLoad(myFortius->getLoad());
    rtData.setSlope(myFortius->getGradient());
}

void FortiusController::pushRealtimeData(RealtimeData &) { } // update realtime data with current values

void
FortiusController::setLoad(double load)
{
    myFortius->setLoad(load);
}

void
FortiusController::setGradientWithSimState(double gradient, double resistanceNewtons, double speedKph)
{
    myFortius->setGradientWithSimState(gradient, resistanceNewtons, speedKph);
}

void
FortiusController::setMode(int mode)
{
    if (mode == RT_MODE_ERGO) mode = Fortius::FT_ERGOMODE;
    else if (mode == RT_MODE_SPIN) mode = Fortius::FT_SSMODE;
    else mode = Fortius::FT_IDLE;
    
    myFortius->setMode(mode);
}

void
FortiusController::setWeight(double weight)
{
    myFortius->setWeight(weight);
}

void
FortiusController::setWindSpeed(double ws)
{
    myFortius->setWindSpeed(ws);
}

void
FortiusController::setRollingResistance(double rr)
{
    myFortius->setRollingResistance(rr);
}

void
FortiusController::setWindResistance(double wr)
{
    myFortius->setWindResistance(wr);
}


// Calibration

uint8_t
FortiusController::getCalibrationType()
{
    return CALIBRATION_TYPE_FORTIUS;
}

double
FortiusController::getCalibrationTargetSpeed()
{
    return 20;
}

uint8_t
FortiusController::getCalibrationState()
{
    return calibrationState;
}

void
FortiusController::setCalibrationState(uint8_t state)
{
    calibrationState = state;
    switch (state)
    {
    case CALIBRATION_STATE_IDLE:
        myFortius->setMode(Fortius::FT_IDLE);
        break;

    case CALIBRATION_STATE_PENDING:
        myFortius->setMode(Fortius::FT_CALIBRATE);
        calibrationState = CALIBRATION_STATE_STARTING;
        break;

    default:
        break;
    }
}

// I don't know if this is the right hook to use
// or if I should be adding a new, better suited function
uint16_t
FortiusController::getCalibrationZeroOffset()
{
    switch (calibrationState)
    {
        // Waiting for use to kick pedal...
        case CALIBRATION_STATE_STARTING:
        {
            if (myFortius->getTelemetry().SpeedMS * 3.6 > 19.9)
            {
                calibration_values.reset();
                calibrationState = CALIBRATION_STATE_STARTED;
            }
            return 0;
        }

        // Calibration running
        case CALIBRATION_STATE_STARTED:
        {
            // keep a note of the last N calibration values
            // keep running calibration until the last N values differ by less than some threshold M

            // Get current value and push onto the list of recent values 
            double latest = myFortius->getTelemetry().ForceNewtons;

            // unexpected resistance (pedalling) will cause calibration to terminate
            if (latest > 0)
            {
                calibrationState = CALIBRATION_STATE_FAILURE;
                return 0;
            }

            // calculate the average
            calibration_values.update(latest);
            const double mean   = calibration_values.mean();
            const double stddev = calibration_values.stddev();

            // wait until stddev within threshold
            // perhaps this isn't the best numerical solution to detect settling
            // but it's better than the previous attempt which was based on diff(min/max)
            // I'd prefer a tighter threshold, eg 0.02
            // but runtime would be too long for users, especially from cold
            static const double stddev_threshold = 0.05;

            if (calibration_values.is_full() && stddev < stddev_threshold) // termination (settling) condition
            {
                // accept the current average as the final valibration value
                myFortius->setBrakeCalibrationForce(-mean);
                calibrationState = CALIBRATION_STATE_SUCCESS;
                myFortius->setMode(Fortius::FT_IDLE);
            }

            // Need to return a uint16_t, and TrainSidebar displays to user as raw value
            return 137. * (calibration_values.is_full() ? mean : latest);
        }

        case CALIBRATION_STATE_SUCCESS:
            return 137. * myFortius->getBrakeCalibrationForce();

        default:
            return 0;
    }
}

void
FortiusController::resetCalibrationState()
{
    calibrationState = CALIBRATION_STATE_IDLE;
    myFortius->setMode(Fortius::FT_IDLE);
}
