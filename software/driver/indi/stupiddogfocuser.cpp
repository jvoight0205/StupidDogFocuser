/*
    Stupid Dog Focuser
    Stupid Dog Observatory
    Copyright 2020 Jeff Voight (jeff.voight@gmail.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 */
#include "stupiddogfocuser.h"

#include "indicom.h"

#include <memory>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#define FOCUSER_MAX_CMD  9
#define FOCUSER_TIMEOUT  3

#define SETTINGS_TAB "Settings"

static std::unique_ptr<StupidDogFocuser> stupidDogFocuser(new StupidDogFocuser());

void ISGetProperties(const char *dev) {
    stupidDogFocuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    stupidDogFocuser->ISNewSwitch(dev, name, states, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    stupidDogFocuser->ISNewNumber(dev, name, values, names, n);

}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
        char *names[], int n) {

    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle *root) {

    stupidDogFocuser->ISSnoopDevice(root);
}

StupidDogFocuser::StupidDogFocuser() {

    FI::SetCapability(
            FOCUSER_CAN_ABS_MOVE |
            FOCUSER_CAN_REL_MOVE |
            FOCUSER_CAN_ABORT |
            FOCUSER_CAN_SYNC |
            FOCUSER_HAS_VARIABLE_SPEED |
            FOCUSER_CAN_REVERSE);
    setSupportedConnections(CONNECTION_SERIAL);
    serialConnection = new Connection::Serial(this);
    //serialConnection->registerHandshake([&]() { return Handshake(); });
    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);

    LOG_INFO("Initialized.");
}

bool StupidDogFocuser::initProperties() {
    INDI::Focuser::initProperties();


    IUFillSwitch(&MicrostepModeS[M32], "M32", "1/32 Step", ISS_OFF);
    IUFillSwitch(&MicrostepModeS[M16], "M16", "1/16 Step", ISS_OFF);
    IUFillSwitch(&MicrostepModeS[M8], "M8", "1/8 Step", ISS_OFF);
    IUFillSwitch(&MicrostepModeS[M4], "M4", "1/4 Step", ISS_OFF);
    IUFillSwitch(&MicrostepModeS[M2], "M2", "1/2 Step", ISS_OFF);
    IUFillSwitch(&MicrostepModeS[M1], "M1", "Full Step", ISS_ON);
    IUFillSwitchVector(&MicrostepModeSP, MicrostepModeS, 6, getDeviceName(), "Microstep Mode", "", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    IUFillLight(&MovingL[0], "Is Moving?", "", IPS_IDLE);
    IUFillLightVector(&MovingLP, MovingL, 1, getDeviceName(), "MOVING", "Moving",
            MAIN_CONTROL_TAB, IPS_OK);

    /* Focuser temperature */
    IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Celsius", "%6.2f", -60., 190., 0., 10000.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature",
            MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Enabled
    IUFillSwitch(&Enabled[0], "ENABLE", "Enable", ISS_ON);
    IUFillSwitch(&Enabled[1], "DISABLE", "Disable", ISS_OFF);
    IUFillSwitchVector(&EnabledSP, Enabled, 2, getDeviceName(), "ENABLE", "Motor Power",
            SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    FocusSpeedN[0].value = 255.0;
    /* Relative and absolute movemennt */

    FocusAbsPosN[0].min = 0;
    FocusAbsPosN[0].max = 108000;
    FocusAbsPosN[0].value = 10800;
    FocusAbsPosN[0].step = 0;
    
    FocusRelPosN[0].max = 10000.0;
    FocusRelPosN[0].value = 1000.0;
    
    FocusMaxPosN[0].min = 0.0;
    FocusMaxPosN[0].max = 999999999.0;
    //FocusMaxPosN[0].step = 1000.0;
    FocusMaxPosN[0].value = 1080000.0;
    
    IDSetNumber(&FocusAbsPosNP, nullptr);
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusMaxPosNP, nullptr);
    simulatedTemperature = 600.0;
    simulatedPosition = 20000;


    addDebugControl();
    addSimulationControl();

    return true;
}

bool StupidDogFocuser::updateProperties() {
    INDI::Focuser::updateProperties();

    if (isConnected()) {
        defineLight(&MovingLP);
        defineNumber(&TemperatureNP);
        defineSwitch(&MicrostepModeSP);
        defineSwitch(&EnabledSP);

        GetFocusParams();
        LOG_DEBUG("StupidDogFocuser parameters readout complete, focuser ready for use.");
    } else {

        deleteProperty(MovingLP.name);
        deleteProperty(TemperatureNP.name);
        deleteProperty(MicrostepModeSP.name);
        deleteProperty(EnabledSP.name);

        LOG_DEBUG("StupidDogFocuser parameters cleared.");
    }

    return true;
}

bool StupidDogFocuser::Handshake() {
    INDI::Focuser::Handshake();

    char firmware[MAX_BUFFER];
    char reply[MAX_BUFFER];

    if (isSimulation()) {
        timerID = SetTimer(POLLMS);
        LOG_INFO("Stupid Dog Focuser Simulator: online. Getting focus parameters...");
        FocusAbsPosN[0].value = simulatedPosition;
        return true;
    }
    tcflush(PortFD, TCIOFLUSH);

    if ((readFocuserFirmware(firmware, reply)) < 0) {
        /* This would be the end*/
        LOGF_ERROR("Unknown error while reading firmware: %s", reply);

        return false;
    }

    return true;
}

const char *StupidDogFocuser::getDefaultName() {

    return "Stupid Dog Focuser";
}

int StupidDogFocuser::sendCommand(char * cmd, char * reply) {

    // Prepare the write
    int nbytes_written = 0;
    int rc = -1;
    int nbytes_read = 0;

    snprintf(commandBuffer, MAX_BUFFER, "<%s>\n", cmd);

    LOGF_DEBUG("Writing command \"%s\"", commandBuffer);

    if ((rc = tty_write_string(PortFD, commandBuffer, &nbytes_written)) != TTY_OK) {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("SERIAL WRITE ERROR: %s.", errstr);
        return -1;
    }
    LOG_DEBUG("Finished writing command");

    LOG_DEBUG("Reading response.");
    if ((rc = tty_read_section(PortFD, responseBuffer, '!', ML_TIMEOUT, &nbytes_read)) != TTY_OK) {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("SERIAL READ ERROR: %s. '%s' didn't end with !.", errstr, responseBuffer);
        //tcflush(PortFD, TCIOFLUSH); // TODO: Remove this flush
        return false;
    }

    char *bangPosit = strchr(responseBuffer, '!');
    bangPosit[0] = '\0';
    //responseBuffer[strlen(responseBuffer) - 1] = '\0';
    LOGF_DEBUG("Response from command '%s'", responseBuffer);
    strcpy(reply, responseBuffer);

    return nbytes_read - 1;
}

int StupidDogFocuser::readFocuserPosition() {
    int temp;

    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;

    LOG_DEBUG("Querying Position...");

    if (isSimulation()) {
        FocusAbsPosN[0].value = simulatedPosition;
        return 0;
    }

    strncpy(focuser_cmd, GET_POSITION, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;

    if (sscanf(focuser_reply, SIGNED_RESPONSE, &temp) < 1)
        return -1;

    FocusAbsPosN[0].value = temp;

    LOGF_DEBUG("Position: %g", temp);

    return 0;
}

int StupidDogFocuser::readFocuserMoving() {
    LOG_DEBUG("Reading Moving");

    bool temp;
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;
    if (isSimulation()) {
        MovingL[0].s = simulatedMoving ? IPS_BUSY : IPS_IDLE;
        return 0;
    }

    strncpy(focuser_cmd, IS_MOVING, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0) {
        MovingLP.s = IPS_ALERT;
        return focuser_rc;
    }

    MovingLP.s = IPS_OK;
    LOGF_DEBUG("Moving response: %s", focuser_reply);
    if (strcmp(focuser_reply, TRUE_RESPONSE) == 0)
        MovingL[0].s = IPS_BUSY;
    else
        MovingL[0].s = IPS_IDLE;

    return 0;
}

int StupidDogFocuser::readFocuserTemperature() {
    LOG_DEBUG("Reading Temperature");
    if (isSimulation()) {
        TemperatureN[0].value = -99.9;
        return 0;
    }

    int temp;
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;

    strncpy(focuser_cmd, GET_TEMPERATURE, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;


    if (sscanf(focuser_reply, SIGNED_RESPONSE, &temp) < 1)
        return -1;

    TemperatureN[0].value = (double) temp;
    LOGF_DEBUG("Temperature: %d", temp);

    return 0;
}

int StupidDogFocuser::readFocuserSpeed() {
    LOG_DEBUG("Reading Speed");
    if (isSimulation()) {
        FocusSpeedN[0].value = 255;
        return 0;
    }

    unsigned int temp;
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;


    strncpy(focuser_cmd, GET_SPEED, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;


    if (sscanf(focuser_reply, UNSIGNED_RESPONSE, &temp) < 1)
        return -1;

    FocusSpeedN[0].value = (double) speed;
    LOGF_DEBUG("Speed: %d", speed);

    return 0;
}

int StupidDogFocuser::readFocuserMicrostep() {
    LOG_DEBUG("Reading Microstep");
    if (isSimulation()) {
        MicrostepModeS[0].s = ISS_ON;
        return 0;
    }

    unsigned int temp;
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;


    strncpy(focuser_cmd, GET_MICROSTEP, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;


    if (sscanf(focuser_reply, UNSIGNED_RESPONSE, &temp) < 1)
        return -1;

    //    switch (temp) {
    //        case 1:
    //            MicrostepModeS[M1].s = ISS_ON;
    //            break;
    //        case 2:
    //            MicrostepModeS[M2].s = ISS_ON;
    //            break;
    //        case 4:
    //            MicrostepModeS[M4].s = ISS_ON;
    //            break;
    //        case 8:
    //            MicrostepModeS[M8].s = ISS_ON;
    //            break;
    //        case 16:
    //            MicrostepModeS[M16].s = ISS_ON;
    //            break;
    //        case 32:
    //            MicrostepModeS[M32].s = ISS_ON;
    //            break;
    //        default:
    //            //No change
    //            break;
    //    }

    return 0;
}

int StupidDogFocuser::readFocuserFirmware(char *_focuser_cmd, char *_focuser_reply) {
    int focuser_rc;

    LOG_DEBUG("Querying Stupid Dog Focuser Firmware...");

    strncpy(_focuser_cmd, GET_VERSION, 5);

    if ((focuser_rc = sendCommand(_focuser_cmd, _focuser_reply)) < 0)
        return focuser_rc;

    if (isSimulation())
        strncpy(_focuser_reply, "SIM", 4);

    LOGF_INFO("Received firmware response: '%s'", _focuser_reply);

    return 0;
}

int StupidDogFocuser::sendFocuserPositionAbsolute(uint32_t value) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;
    int newPosition;

    LOGF_DEBUG("Moving Absolute Position: %ld", (long) value);

    if (isSimulation()) {
        simulatedPosition = value;
        return 0;
    }

    snprintf(focuser_cmd, MAX_BUFFER, ABSOLUTE_MOVE, (long) value);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;


    if (sscanf(focuser_reply, SIGNED_RESPONSE, &newPosition) < 1)
        return -1;
    readFocuserPosition();

    return 0;
}

bool StupidDogFocuser::SyncFocuser(uint32_t ticks) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int ret_read_tmp;

    if (isSimulation()) {
        FocusAbsPosN[0].value = ticks;
        return true;
    }

    snprintf(focuser_cmd, MAX_BUFFER, SYNC_MOTOR, (long) ticks);


    if ((ret_read_tmp = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return false;


    if (sscanf(focuser_reply, SIGNED_RESPONSE, &ticks) < 1)
        return false;

    FocusAbsPosN[0].value = ticks;

    return true;
}

bool StupidDogFocuser::SetFocuserSpeed(int speed) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int ret_read_tmp;

    if (isSimulation()) {
        FocusSpeedN[0].value = speed;
        return true;
    }

    snprintf(focuser_cmd, MAX_BUFFER, SET_SPEED, speed);


    if ((ret_read_tmp = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return false;


    if (sscanf(focuser_reply, UNSIGNED_RESPONSE, &ret_read_tmp) < 1)
        return false;

    FocusSpeedN[0].value = ret_read_tmp;

    return true;
}

bool StupidDogFocuser::SetFocuserMicrostep(int _microstep) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int ret_read_tmp;
    if (_microstep != 1 && _microstep != 2 && _microstep != 4 && _microstep != 8 && _microstep != 16 && _microstep != 32) {
        LOGF_ERROR("Unable to set microstep to %d.", _microstep);
        MicrostepModeSP.s = IPS_ALERT;
        return false;
    }
    if (isSimulation()) {
        return true;
    }

    snprintf(focuser_cmd, MAX_BUFFER, SET_MICROSTEP, _microstep);


    if ((ret_read_tmp = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return false;


    if (sscanf(focuser_reply, UNSIGNED_RESPONSE, &ret_read_tmp) < 1)
        return false;

    return true;
}

/*
 * If the user changes a text element, make sure that we notify the instance.
 */

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
    stupidDogFocuser->ISNewText(dev, name, texts, names, n);
}

/*
 * If the user changes a switch, make sure that we notify the instance
 */
bool StupidDogFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    // Focus Step Mode
    //    if (strcmp(MicrostepModeSP.name, name) == 0) {
    //        int current_mode = IUFindOnSwitchIndex(&MicrostepModeSP);
    //        int new_mode = current_mode;
    //        
    //        IUUpdateSwitch(&MicrostepModeSP, states, names, n);
    //
    //        int target_mode = IUFindOnSwitchIndex(&MicrostepModeSP);
    //
    //        if (current_mode == target_mode) {
    //            MicrostepModeSP.s = IPS_OK;
    //            IDSetSwitch(&MicrostepModeSP, nullptr);
    //            return true;
    //        }
    //
    //        switch(target_mode){
    //            case 0:
    //                new_mode = 1;
    //                break;
    //            case 1:
    //                new_mode = 2;
    //                break;
    //            case 2:
    //                new_mode = 4;
    //                break;
    //            case 3:
    //                new_mode = 8;
    //                break;
    //            case 4:
    //                new_mode = 16;
    //                break;
    //            case 5:
    //                new_mode = 32;
    //                break;
    //            default:
    //                new_mode = -1;
    //        }
    //
    //        if (new_mode == -1 || !SetFocuserMicrostep(new_mode)) {
    //            IUResetSwitch(&MicrostepModeSP);
    //            MicrostepModeS[current_mode].s = ISS_ON;
    //            MicrostepModeSP.s = IPS_ALERT;
    //            IDSetSwitch(&MicrostepModeSP, nullptr);
    //        }
    //        
    //        MicrostepModeSP.s = IPS_OK;
    //        IDSetSwitch(&MicrostepModeSP, nullptr);
    //        return true;
    //    }
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

/*
 * If the user changes a number or slider, make sure that we notify the instance
 */
bool StupidDogFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

void StupidDogFocuser::GetFocusParams() {
    int ret = -1;
    LOG_DEBUG("Getting Focus Params.");
    tcflush(PortFD, TCIOFLUSH);

    // Is moving
    if ((ret = readFocuserMoving()) < 0) {
        MovingLP.s = IPS_ALERT;
        LOGF_ERROR("Unknown error while reading Stupid Dog Focuser isMoving: %d", ret);
        IDSetLight(&MovingLP, nullptr);
        return;
    }

    IDSetLight(&MovingLP, nullptr);

    if ((ret = readFocuserPosition()) < 0) {
        FocusAbsPosNP.s = IPS_ALERT;
        LOGF_ERROR("Unknown error while reading Stupid Dog Focuser position: %d", ret);
        IDSetNumber(&FocusAbsPosNP, nullptr);
        return;
    }

    FocusAbsPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    if ((ret = readFocuserTemperature()) < 0) {
        TemperatureNP.s = IPS_ALERT;
        LOG_ERROR("Unknown error while reading Stupid Dog Focuser temperature.");
        IDSetNumber(&TemperatureNP, nullptr);
        return;
    }

    TemperatureNP.s = IPS_OK;
    IDSetNumber(&TemperatureNP, nullptr);

    // Speed
    if ((ret = readFocuserSpeed()) < 0) {
        FocusSpeedNP.s = IPS_ALERT;
        LOGF_ERROR("Unknown error while reading Stupid Dog Focuser speed: %d", ret);
        IDSetNumber(&FocusSpeedNP, nullptr);
        return;
    }

    FocusSpeedNP.s = IPS_OK;
    FocusSpeedN[0].value = speed;

    IDSetNumber(&FocusSpeedNP, nullptr);

    // Microstep
    if ((ret = readFocuserMicrostep()) < 0) {
        MicrostepModeSP.s = IPS_ALERT;
        LOGF_ERROR("Unknown error while reading Stupid Dog Focuser microstep: %d", ret);
        IDSetSwitch(&MicrostepModeSP, nullptr);
        return;
    }

    MicrostepModeSP.s = IPS_OK;
    IDSetSwitch(&MicrostepModeSP, nullptr);
    
    FocusAbortSP.s = IPS_OK;
    IDSetSwitch(&FocusAbortSP, nullptr);
    // Is Reversed
}

IPState StupidDogFocuser::MoveAbsFocuser(uint32_t targetTicks) {
    int ret = -1;
    targetPos = targetTicks;

    if (targetTicks > FocusAbsPosN[0].max) {
        LOG_ERROR("Error, requested position is out of range.");
        return IPS_ALERT;
    }

    if (ret = sendFocuserPositionAbsolute(targetTicks) < 0) {
        LOGF_ERROR("Sending the absolute movement failed %3d", ret);
        return IPS_ALERT;
    }

    MovingL[0].s = IPS_BUSY;

    return IPS_OK;
}

IPState StupidDogFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks) {

    return MoveAbsFocuser(FocusAbsPosN[0].value + (ticks * (dir == FOCUS_INWARD ? -1 : 1)));
}

bool StupidDogFocuser::AbortFocuser() {
    LOG_INFO("Aborting focuser!");

    int nbytes_written;
    const char *buf = "<HA>\r";

    return tty_write(PortFD, buf, strlen(buf), &nbytes_written) == TTY_OK;
}

void StupidDogFocuser::TimerHit() {
    if (!isConnected()) {
        SetTimer(POLLMS);
        return;
    }
    GetFocusParams();
    SetTimer(POLLMS);
}

