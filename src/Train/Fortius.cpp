/*
 * Copyright (c) 2011 Mark Liversedge (liversedge@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "Fortius.h"

// From switchabl on FortAnt project, based on 
//  https://github.com/totalreverse/ttyT1941/wiki#newer-usb-1264-bytes-protocol
//
// Power resistance factor is 137 * 289.75 * 3.6 ~= 142905
static const double s_kphFactorMS = 3.6;
static const double s_newtonsToResistanceFactor = 137;
static const double s_deviceSpeedFactorMS = 289.75 * s_kphFactorMS; 

class Lock
{
    QMutex& mutex;
public:
    Lock(QMutex& m) : mutex(m) { mutex.lock(); }
    ~Lock() { mutex.unlock(); }
};
    
/* ----------------------------------------------------------------------
 * CONSTRUCTOR/DESRTUCTOR
 * ---------------------------------------------------------------------- */
Fortius::Fortius(QObject *parent) : QThread(parent)
{
    
    deviceForceNewtons = devicePowerWatts = deviceHeartRate = deviceCadence = deviceSpeedMS = 0.00;
    mode = FT_IDLE;
    loadWatts = DEFAULT_LOAD;
    gradient = DEFAULT_GRADIENT;
    weight = DEFAULT_WEIGHT;
    brakeCalibrationFactor = DEFAULT_CALIBRATION;
    powerScaleFactor = DEFAULT_SCALING;
    deviceStatus=0;
    this->parent = parent;

    // for interacting over the USB port
    usb2 = new LibUsb(TYPE_FORTIUS);
}

Fortius::~Fortius()
{
    delete usb2;
}

/* ----------------------------------------------------------------------
 * SET
 * ---------------------------------------------------------------------- */
void Fortius::setMode(int mode)
{
    Lock lock(pvars);
    this->mode = mode;
}

// Alters the relationship between brake setpoint at load.
void Fortius::setBrakeCalibrationFactor(double brakeCalibrationFactor)
{
    Lock lock(pvars);
    this->brakeCalibrationFactor = brakeCalibrationFactor;
}

// output power adjusted by this value so user can compare with hub or crank based readings
void Fortius::setPowerScaleFactor(double powerScaleFactor)
{
    if (powerScaleFactor < 0.8) powerScaleFactor = 0.8;
    if (powerScaleFactor > 1.2) powerScaleFactor = 1.2;
    
    Lock lock(pvars);
    this->powerScaleFactor = powerScaleFactor;
}

// User weight used by brake in slope mode
void Fortius::setWeight(double weight)
{
    // need to apply range as same byte used to signify erg mode
    if (weight < 50) weight = 50;
    if (weight > 120) weight = 120;
        
    Lock lock(pvars);
    this->weight = weight;
}

// Load in watts when in power mode
void Fortius::setLoad(double loadWatts)
{
    Lock lock(pvars);
    this->loadWatts = loadWatts;
}

// Resistance in newtons when implementing 'slope' mode
void Fortius::setGradientWithSimState(double gradient, double resistanceNewtons, double speedKph)
{
    Lock lock(pvars);
    this->resistanceNewtons = resistanceNewtons;
    this->simSpeedMS = speedKph / s_kphFactorMS; // for aligning simulator and trainer speeds
    this->gradient = gradient; // Eye candy, eventually remove. Not used to set load.
}

/* ----------------------------------------------------------------------
 * GET
 * ---------------------------------------------------------------------- */
void Fortius::getTelemetry(double &powerWatts, double &heartrate, double &cadence, double &speedKph, double &distance, int &buttons, int &steering, int &status)
{
    Lock lock(pvars);
    powerWatts = devicePowerWatts;
    heartrate = deviceHeartRate;
    cadence = deviceCadence;
    speedKph = deviceSpeedMS * s_kphFactorMS;
    distance = deviceDistance;
    buttons = deviceButtons;
    steering = deviceSteering;
    status = deviceStatus;
    
    // work around to ensure controller doesn't miss button press. 
    // The run thread will only set the button bits, they don't get
    // reset until the ui reads the device state
    deviceButtons = 0; 
}

int Fortius::getMode() const
{
    Lock lock(pvars);
    return mode;
}

double Fortius::getLoad() const
{
    Lock lock(pvars);
    return this->loadWatts;
}

double Fortius::getGradient() const
{
    Lock lock(pvars);
    return gradient;
}

double Fortius::getWeight() const
{
    Lock lock(pvars);
    return weight;
}

double Fortius::getBrakeCalibrationFactor() const
{
    Lock lock(pvars);
    return brakeCalibrationFactor;
}

double Fortius::getPowerScaleFactor() const
{
    Lock lock(pvars);
    return powerScaleFactor;
}

int
Fortius::start()
{
    {
        Lock lock(pvars);
        this->deviceStatus = FT_RUNNING;
    }

    QThread::start();
    return 0;
}



/* ----------------------------------------------------------------------
 * EXECUTIVE FUNCTIONS
 *
 * start() - start/re-start reading telemetry in a thread
 * stop() - stop reading telemetry and terminates thread
 * pause() - discards inbound telemetry (ignores it)
 *
 *
 * THE MEAT OF THE CODE IS IN RUN() IT IS A WHILE LOOP CONSTANTLY
 * READING TELEMETRY AND ISSUING CONTROL COMMANDS WHILST UPDATING
 * MEMBER VARIABLES AS TELEMETRY CHANGES ARE FOUND.
 *
 * run() - bg thread continuosly reading/writing the device port
 *         it is kicked off by start and then examines status to check
 *         when it is time to pause or stop altogether.
 * ---------------------------------------------------------------------- */
int Fortius::restart()
{
    // get current status
    Lock lock(pvars);

    int status = this->deviceStatus;

    // what state are we in anyway?
    if (status&FT_RUNNING && status&FT_PAUSED) {
        status &= ~FT_PAUSED;
        this->deviceStatus = status;
        return 0; // ok its running again!
    }
    return 2;
}

int Fortius::stop()
{
    // what state are we in anyway?
    Lock lock(pvars);
    deviceStatus = 0; // Terminate it!
    return 0;
}

int Fortius::pause()
{
    Lock lock(pvars);

    // get current status
    int status = this->deviceStatus;

    if (status&FT_PAUSED) return 2; // already paused you muppet!
    else if (!(status&FT_RUNNING)) return 4; // not running anyway, fool!
    else {
        // ok we're running and not paused so lets pause
        status |= FT_PAUSED;
        this->deviceStatus = status;

        return 0;
    }
}

// used by thread to set variables and emit event if needed
// on unexpected exit
int Fortius::quit(int code)
{
    // event code goes here!
    exit(code);
    return 0; // never gets here obviously but shuts up the compiler!
}

/*----------------------------------------------------------------------
 * THREADED CODE - READS TELEMETRY AND SENDS COMMANDS TO KEEP FORTIUS ALIVE
 *----------------------------------------------------------------------*/
void Fortius::run()
{

    // newly read values - compared against cached values
    bool isDeviceOpen = false;

    // ui controller state
    int curstatus;

    // variables for telemetry, copied to fields on each brake update
    double curForceNewtons;               // current output power in Watts
    double curPowerWatts;                 // current output power in Watts
    double curHeartRate;                  // current heartrate in BPM
    double curCadence;                    // current cadence in RPM
    double curSpeedMS;                    // current speed in meters per second
    // UNUSED double curDistance;                    // odometer?
    int curButtons;                       // Button status
    int curSteering;                    // Angle of steering controller
    // UNUSED int curStatus;
    uint8_t pedalSensor;                // 1 when using is cycling else 0, fed back to brake although appears unnecessary

    // we need to average out power for the last second
    // since we get updates every 10ms (100hz)
    int powerhist[10];     // last 10 values received
    int powertot=0;        // running total
    int powerindex=0;      // index into the powerhist array
    for (int i=0; i<10; i++) powerhist[i]=0; 
                                        
    // initialise local cache & main vars
    {
        Lock lock(pvars);

        // UNUSED curStatus = this->deviceStatus;
        curForceNewtons = this->deviceForceNewtons = 0;
        curPowerWatts = this->devicePowerWatts = 0;
        curHeartRate = this->deviceHeartRate = 0;
        curCadence = this->deviceCadence = 0;
        curSpeedMS = this->deviceSpeedMS = 0;
        // UNUSED curDistance = this->deviceDistance = 0;
        curSteering = this->deviceSteering = 0;
        curButtons = this->deviceButtons = 0;
        pedalSensor = 0;
    }

    // open the device
    if (openPort()) {
        quit(2);
        return; // open failed!
    } else {
        isDeviceOpen = true;
        sendToTrainer(Command_OPEN());
    }

    QTime timer;
    timer.start();

    while(1) {

        if (isDeviceOpen == true) {

			int rc = sendRunCommand(pedalSensor) ;
			if (rc < 0) {
				qDebug() << "usb write error " << rc;
				// send failed - ouch!
				closePort(); // need to release that file handle!!
				quit(4);
        		return; // couldn't write to the device
			}
			
            int actualLength = readMessage();
			if (actualLength < 0) {
				qDebug() << "usb read error " << actualLength;
			}
            if (actualLength >= 24) {

                //----------------------------------------------------------------
                // UPDATE BASIC TELEMETRY (HR, CAD, SPD et al)
                // The data structure is very simple, no bit twiddling needed here
                //----------------------------------------------------------------
                // The first 12 bytes are almost always identical, with buf[10] being
                // exceptionally 40 instead of 50:
                //
                // 20 9f 00 00 08 01 00 00 07 00 50 bd
                //
                // buf[12] is heart rate
                // buf[13] is buttons
                // buf[14, 15] change from time to time, 00 00 or 00 01 (controller status?)
                // buf[16, 17] remain around 6d 02
                // buf[18, 19] is the steering
                // buf[20 - 27] remain constant at d0 07 d0 07 03 13 02 00
                // buf[28 - 31] is the distance
                // buf[32, 33] is the speed
                // buf[34] varies around 2c
                // buf[35] jumps mostly between 02 and 0e.
                // buf[36, 37] vary with the speed but go to 0 more quickly than the speed or power
                // buf[38, 39] is the torque output of the cyclist
                // buf[40, 41] also vary somewhat with speed
                // buf[42] pedal sensor, normally 0, 1 when the pedal passes near the sensor
                // buf[43] remains 0
                // buf[44, 45] cadence
                // buf[46] is 0x02 when active, 0x00 at the end
                // buf[47] varies even when the system is idle
                
                // buttons
                curButtons = buf[13];

                // steering angle
                curSteering = buf[18] | (buf[19] << 8);
                
                // update public fields
                {
                    Lock lock(pvars);
                    deviceButtons |= curButtons;    // workaround to ensure controller doesn't miss button pushes
                    deviceSteering = curSteering;
                }
            }
            if (actualLength >= 48) {
                // brake status status&0x04 == stopping wheel
                //              status&0x01 == brake on
                //curBrakeStatus = buf[46?];
                
                // pedal sensor is 0x01 when cycling
                pedalSensor = buf[42];
                
                // UNUSED curDistance = (buf[28] | (buf[29] << 8) | (buf[30] << 16) | (buf[31] << 24)) / 16384.0;

                curCadence = buf[44];
				
                // speed

                curSpeedMS = qFromLittleEndian<quint16>(&buf[32]) / s_deviceSpeedFactorMS;

                // Power is torque * wheelspeed - adjusted by device resistance factor.
                curForceNewtons = qFromLittleEndian<qint16>(&buf[38]) / s_newtonsToResistanceFactor;

                curPowerWatts = curForceNewtons * curSpeedMS;

                { // braces, just to collect power adjustments together, visually
                    if (curPowerWatts < 0.0) curPowerWatts = 0.0;  // brake power can be -ve when coasting. 
                
                    // average power over last 10 readings
                    powertot += curPowerWatts;
                    powertot -= powerhist[powerindex];
                    powerhist[powerindex] = curPowerWatts;

                    curPowerWatts = powertot / 10;
                    powerindex = (powerindex == 9) ? 0 : powerindex+1; 

                    curPowerWatts *= powerScaleFactor; // apply scale factor
                }

                curHeartRate = buf[12];

                // update public fields
                {
                    Lock lock(pvars);
                    deviceSpeedMS = curSpeedMS;
                    deviceCadence = curCadence;
                    deviceHeartRate = curHeartRate;
                    deviceForceNewtons = curForceNewtons;
                    devicePowerWatts = curPowerWatts;
                }
            }
        }

        //----------------------------------------------------------------
        // LISTEN TO GUI CONTROL COMMANDS
        //----------------------------------------------------------------
        {
            Lock lock(pvars);
            curstatus = this->deviceStatus;
        }

        /* time to shut up shop */
        if (!(curstatus&FT_RUNNING)) {
            // time to stop!
            
            sendToTrainer(Command_CLOSE());
            
            closePort(); // need to release that file handle!!
            quit(0);
            return;
        }

        if ((curstatus&FT_PAUSED) && isDeviceOpen == true) {
        
            closePort();
            isDeviceOpen = false;

        } else if (!(curstatus&FT_PAUSED) && (curstatus&FT_RUNNING) && isDeviceOpen == false) {

            if (openPort()) {
                quit(2);
                return; // open failed!
            }
            isDeviceOpen = true;        
            sendToTrainer(Command_OPEN());
                        
            timer.restart();
        }

        
        // The controller updates faster than the brake. Setting this to a low value (<50ms) increases the frequency of controller
        // only packages (24byte). Tacx software uses 100ms.
        msleep(50);
    }
}

/* ----------------------------------------------------------------------
 * HIGH LEVEL DEVICE IO AND COMMAND ENCODING ROUTINES
 *
 * sendRunCommand(int) - update brake setpoint
 *
 * Commmand_OPEN()         - returns message used to start device
 * Commmand_GENERIC(...)   - returns 12 byte message to control trainer
 *                         - common to all Command_XXX() functions below
 * Commmand_CLOSE()        - returns message to put device in idle mode
 * Commmand_ERGO(...)      - returns message to set ERGO resistance (no flywheel)
 * Commmand_SLOPE(...)     - returns message to set SLOPE resistance (with flywheel)
 * Commmand_CALIBRATE(...) - returns message to put trainer in calibration mode
 * 
 *
 * ---------------------------------------------------------------------- */

namespace {
    // Some helper functions in anonymous namespace
    // These are only used in Fortius::sendRunCommand(), immediately below
    // They are the functions we will use to modify behaviour wrt accuracy, feel, limits, etc

    // ------------------------------------------------------------
    // Provide a pair of points on device max power graph, this
    // class provides a function to tell you max force at speed.
    //
    // Only supports linear max power graph.
    //
    class MaxForceAtSpeed {
        double m, b;

        public:

        MaxForceAtSpeed(double ms0, double watts0, double ms1, double watts1) {
            m = (watts1 - watts0) / (ms1 - ms0);
            b = watts0 - (m * ms0);
        }

        double maxForce(double ms) const {
            return (ms < 0.1) ? 0 : (m * ms + b) / ms;
        }
    };

    // Using iFlow force limits because they are lowest.
    // Need ui work to support multiple devices.
    static const MaxForceAtSpeed s_iFlow  (0, 0, 60 / s_kphFactorMS, 800);
    static const MaxForceAtSpeed s_iVortex(0, 0, 60 / s_kphFactorMS, 900);
    static const MaxForceAtSpeed s_iGenius(0, 0, 38 / s_kphFactorMS, 1200);
    static const MaxForceAtSpeed s_Bushido(0, 0, 40 / s_kphFactorMS, 1200);

    // Ensure that force never exceeds physical limit of device.
    double LimitResistanceNewtons(double speedMS, double newtons, const MaxForceAtSpeed& model = s_iGenius) {
        return std::max<double>(std::min<double>(newtons, model.maxForce(speedMS)), -5);
    }


    // ------------------------------------------------------------
    // TODO: Ensure this filtering is adequately handled in
    //       LimitResistanceNewtons function, then remove.
    //
    // FortiusAnt Speed Sensitive Resistance Limits.
    //
    // The fortius power range only applies at high rpm. The device cannot
    // hold against the torque of high power at low rpm.
    //
    // This function caps power at low trainer speed, if you want more power
    // you should use a higher gear to drive trainer faster.
    double WoutersLowSpeedLimit(double deviceSpeedMS, double requestedForceN)
    {
        // def __AvoidCycleOfDeath(self, Resistance):
        //   if self.TargetMode == mode_Power and self.SpeedKmh <= 10 and Resistance >= 6000:
        //     Resistance = int(1500 + self.SpeedKmh * 300)

        double deviceResistance     = requestedForceN * s_newtonsToResistanceFactor;
        const double deviceSpeedKPH = deviceSpeedMS * s_kphFactorMS;

        if (deviceSpeedKPH <= 10 && deviceResistance >= 6000) {
            deviceResistance = 1500 + (deviceSpeedKPH * 300);
        }

        return deviceResistance / s_newtonsToResistanceFactor;
    }
}

int Fortius::sendRunCommand(int16_t pedalSensor)
{
    pvars.lock();
    const int mode = this->mode;
    const double loadWatts = this->loadWatts;
    const double resistanceNewtons = this->resistanceNewtons;
    const double simSpeedMS = this->simSpeedMS;
    const double weight = this->weight;
    const double brakeCalibrationFactor = this->brakeCalibrationFactor;
    pvars.unlock();


    // Ensure that load never exceeds physical limit of device.
    const auto UpperForceLimit = [this](double forceN)
    {
        // Linear (ideal) device limit
        forceN = LimitResistanceNewtons(this->deviceSpeedMS, forceN);

        // Low-wheel-speed (empirical) device limit
        forceN = WoutersLowSpeedLimit  (this->deviceSpeedMS, forceN);
        return forceN;
    };


    // Depending on mode, send the appropriate command to the trainer
    switch (mode)
    {
        case FT_IDLE:
            return sendToTrainer(Command_OPEN());

        case FT_ERGOMODE:
            {
                // Trainer is being instructed to provide (loadWatts)
                // Force (N) = Power (W) / Speed (m/s)
                // Note: avoid divide by zero
                const double targetForceNewtons = loadWatts / std::max(0.1, this->deviceSpeedMS);

                // Send command to trainer
                return sendToTrainer(
                    Command_ERGO(
                        UpperForceLimit(targetForceNewtons),
                        pedalSensor,
                        (130 * brakeCalibrationFactor + 1040)));
            }

        case FT_SSMODE:
            {
                // Slope mode receives newtons of resistance directly.    
                // Send command to trainer
                return sendToTrainer(
                    Command_SLOPE(
                        UpperForceLimit(resistanceNewtons),
                        pedalSensor, weight,
                        (130 * brakeCalibrationFactor + 1040)));
            }

        case FT_CALIBRATE:
            return sendToTrainer(Command_CALIBRATE(20 / s_kphFactorMS));

        default:
            break; // error if here
    }

    return 0;
}


// Outbound control message has the format:
// Byte          Value / Meaning
// 0             0x01 CONSTANT
// 1             0x08 CONSTANT
// 2             0x01 CONSTANT
// 3             0x00 CONSTANT
// 4             Brake Value - Lo Byte
// 5             Brake Value - Hi Byte
// 6             Echo cadence sensor
// 7             0x00 -- UNKNOWN
// 8             0x02 -- 0 - idle, 2 - Active, 3 - Calibration
// 9             0x52 -- Mode 0a = ergo, weight for slope mode (48 = 72kg), 52 = idle (in conjunction with byte 8)
// 10            Calibration Value - Lo Byte
// 11            Calibration High - Hi Byte

// Encoded Calibration is 130 x Calibration Value + 1040 so calibration of zero gives 0x0410

Fortius::ShortTrainerCommand Fortius::Command_OPEN()
{
    return {0x02,0x00,0x00,0x00};
}

Fortius::TrainerCommand Fortius::Command_GENERIC(uint8_t mode, double forceNewtons, uint8_t pedecho, uint8_t weight, uint16_t calibration)
{
    TrainerCommand command = { 0x01, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    const double resistance = std::min<double>(SHRT_MAX, (forceNewtons * s_newtonsToResistanceFactor));
    qToLittleEndian<int16_t>(resistance, &command[4]);

    command[6] = pedecho;
    command[8] = mode;
    command[9] = weight;

    qToLittleEndian<int16_t>(calibration, &command[10]);

    return command;
}

Fortius::TrainerCommand Fortius::Command_CLOSE()
{
    return Command_GENERIC(FT_MODE_IDLE, 0, 0, 0x52 /* flywheel enabled at 82 kg */, 0);
}

Fortius::TrainerCommand Fortius::Command_ERGO(double forceNewtons, uint8_t pedecho, uint16_t calibration)
{
    return Command_GENERIC(FT_MODE_ACTIVE, forceNewtons, pedecho, 0x0a, calibration);
}

Fortius::TrainerCommand Fortius::Command_SLOPE(double forceNewtons, uint8_t pedecho, uint8_t weight, uint16_t calibration)
{
    return Command_GENERIC(FT_MODE_ACTIVE, forceNewtons, pedecho, weight, calibration);
}

Fortius::TrainerCommand Fortius::Command_CALIBRATE(double speedMS)
{
    return Command_GENERIC(FT_MODE_CALIBRATE, speedMS * s_deviceSpeedFactorMS, 0, 0, 0);
}


/* ----------------------------------------------------------------------
 * LOW LEVEL DEVICE IO ROUTINES - PORT TO QIODEVICE REQUIRED BEFORE COMMIT
 *
 *
 * readMessage()        - reads an inbound message
 * openPort() - opens serial device and configures it
 * closePort() - closes serial device and releases resources
 * rawRead() - non-blocking read of inbound data
 * rawWrite() - non-blocking write of outbound data
 * discover() - check if a ct is attached to the port specified
 * ---------------------------------------------------------------------- */
int Fortius::readMessage()
{
    int rc;

    rc = rawRead(buf, 64);
	//qDebug() << "usb status " << rc;
    return rc;
}

int Fortius::closePort()
{
    usb2->close();
    return 0;
}

bool Fortius::find()
{
	int rc;
    rc = usb2->find();
	//qDebug() << "usb status " << rc;
    return rc;
}

int Fortius::openPort()
{
	int rc;
    // on windows we try on USB2 then on USB1 then fail...
    rc = usb2->open();
	//qDebug() << "usb status " << rc;
    return rc;
}

int Fortius::rawWrite(const uint8_t *bytes, int size) // unix!!
{
    return usb2->write((char *)bytes, size, FT_USB_TIMEOUT);
}

int Fortius::rawRead(uint8_t bytes[], int size)
{
    return usb2->read((char *)bytes, size, FT_USB_TIMEOUT);
}

// check to see of there is a port at the device specified
// returns true if the device exists and false if not
bool Fortius::discover(QString)
{
    return true;
}
