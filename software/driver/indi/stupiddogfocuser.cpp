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

#define FOCUSER_MAX_CMD  9
#define FOCUSER_TIMEOUT  3

#define MAXTRAVEL_READOUT 99999

#define currentPosition         FocusAbsPosN[0].value
#define currentTemperature      TemperatureN[0].value
#define currentRelativeMovement FocusRelPosN[0].value
#define currentAbsoluteMovement FocusAbsPosN[0].value
#define currentMinPosition      MinMaxPositionN[0].value
#define currentMaxPosition      MinMaxPositionN[1].value
#define currentMaxTravel        FocusMaxPosN[0].value

#define SETTINGS_TAB "Settings"

#define FEED_GO ":FG#"
#define HALT_MOVE_COMMAND ":FQ#"
#define GET_HALF_STEP ":GH#"
#define GET_ISMOVING ":GI#"
#define GET_NEW_POSITION ":GN#"
#define GET_POSITION ":GP#"
#define GET_TEMPERATURE ":GT#"
#define GET_VERSION ":GV#"
#define SET_FULL_STEP ":SF#"
#define SET_HALF_STEP ":SH#"
#define SET_NEW_POSITION ":SN%u#"
#define SET_CURRENT_POSITION ":SP%u#"

static std::unique_ptr<StupidDogFocuser> stupidDogFocuser(new StupidDogFocuser());

void ISGetProperties(const char *dev) {
    stupidDogFocuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    stupidDogFocuser->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
    stupidDogFocuser->ISNewText(dev, name, texts, names, n);
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
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
    setSupportedConnections(CONNECTION_SERIAL);
}

bool StupidDogFocuser::initProperties() {
    INDI::Focuser::initProperties();

    /* Focuser temperature */
    IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Celsius", "%6.2f", 0., 65000., 0., 10000.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature",
            MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    /* Stupid Dog Focuser should stay within these limits */
    IUFillNumber(&MinMaxPositionN[0], "MINPOS", "Minimum Tick", "%6.0f", 1., 65535., 0., 100.);
    IUFillNumber(&MinMaxPositionN[1], "MAXPOS", "Maximum Tick", "%6.0f", 1., 65535., 0., 50000.);
    IUFillNumberVector(&MinMaxPositionNP, MinMaxPositionN, 2, getDeviceName(), "FOCUS_MINMAXPOSITION", "Extrema",
            SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    IUFillNumber(&MaxTravelN[0], "MAXTRAVEL", "Maximum travel", "%6.0f", 1., 64000., 0., 10000.);
    IUFillNumberVector(&MaxTravelNP, MaxTravelN, 1, getDeviceName(), "FOCUS_MAXTRAVEL", "Max. travel", SETTINGS_TAB,
            IP_RW, 0, IPS_IDLE);

    // Cannot change maximum position
    FocusMaxPosNP.p = IP_RO;
    FocusMaxPosN[0].value = 65535;

    /* Relative and absolute movement */
    FocusRelPosN[0].min = 0.;
    FocusRelPosN[0].max = 5000.;
    FocusRelPosN[0].value = 100;
    FocusRelPosN[0].step = 100;

    FocusAbsPosN[0].min = 0;
    FocusAbsPosN[0].max = 65535;
    FocusAbsPosN[0].value = 10000;
    FocusAbsPosN[0].step = 1000;

    simulatedTemperature = 600.0;
    simulatedPosition = 20000;

    addDebugControl();
    addSimulationControl();

    return true;
}

bool StupidDogFocuser::updateProperties() {
    INDI::Focuser::updateProperties();

    if (isConnected()) {
        defineNumber(&TemperatureNP);
        defineNumber(&MinMaxPositionNP);
        defineNumber(&MaxTravelNP);

        GetFocusParams();

        LOG_DEBUG("StupidDogFocuser parameters readout complete, focuser ready for use.");
    } else {
        deleteProperty(TemperatureNP.name);
        deleteProperty(MinMaxPositionNP.name);
        deleteProperty(MaxTravelNP.name);
    }

    return true;
}

bool StupidDogFocuser::Handshake() {
    char firmware[] = "10";

    if (isSimulation()) {
        timerID = SetTimer(POLLMS);
        LOG_INFO("Stupid Dog Focuser Simulator: online. Getting focus parameters...");
        FocusAbsPosN[0].value = simulatedPosition;
        updateFocuserFirmware(firmware);
        return true;
    }

    if ((updateFocuserFirmware(firmware)) < 0) {
        /* This would be the end*/
        LOG_ERROR("Unknown error while reading firmware.");
        return false;
    }

    return true;
}

const char *StupidDogFocuser::getDefaultName() {
    return "StupidDogFocuser";
}

int StupidDogFocuser::sendCommand(const char * cmd)
{
    int nbytes_written = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);

    LOGF_DEBUG("CMD <%s>", cmd);

    if ((rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return -1;
    }
    
    return nbytes_written;
}

int StupidDogFocuser::readResponse(char *res)
{
    int nbytes_read = 0, rc = -1;

    if ((rc = tty_nread_section(PortFD, res, ML_RES, ML_DEL, ML_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);

    return nbytes_read;
}

int StupidDogFocuser::updateFocuserPosition(double *value) {
    unsigned temp;
    char focuser_cmd[FOCUSER_MAX_CMD];
    int focuser_rc;

    LOG_DEBUG("Querying Position...");

    if (isSimulation()) {
        *value = simulatedPosition;
        return 0;
    }

    strncpy(focuser_cmd, GET_POSITION, 5);

    if ((focuser_rc = sendCommand(focuser_cmd)) < 0)
        return focuser_rc;

    if ((focuser_rc = readResponse(focuser_cmd)) < 0)
        return focuser_rc;

    if (sscanf(focuser_cmd, "%d#", &temp) < 1)
        return -1;

    *value = (double) temp;

    LOGF_DEBUG("Position: %g", *value);

    return 0;
}

int StupidDogFocuser::updateFocuserTemperature(double *value) {
    LOGF_DEBUG("Update Temperature: %g", value);

    float temp;
    char focuser_cmd[32];
    int focuser_rc;

    strncpy(focuser_cmd, GET_TEMPERATURE, 5);

    if ((focuser_rc = sendCommand(focuser_cmd)) < 0)
        return focuser_rc;

    if (isSimulation())
        snprintf(focuser_cmd, 32, "%f#", simulatedTemperature);
    else if ((focuser_rc = readResponse(focuser_cmd)) < 0)
        return focuser_rc;

    if (sscanf(focuser_cmd, "%f#", &temp) < 1)
        return -1;

    *value = temp;

    return 0;
}

int StupidDogFocuser::updateFocuserFirmware(char *_focuser_cmd) {
    int focuser_rc;

    LOG_DEBUG("Querying StupidDogFocuser Firmware...");

    strncpy(_focuser_cmd, GET_VERSION, 5);

    if ((focuser_rc = sendCommand(_focuser_cmd)) < 0)
        return focuser_rc;

    if (isSimulation())
        strncpy(_focuser_cmd, "SIM", 4);
    else if ((focuser_rc = readResponse(_focuser_cmd)) < 0)
        return focuser_rc;

    return 0;
}

int StupidDogFocuser::updateFocuserPositionAbsolute(double value) {
    char focuser_cmd[32];
    int focuser_rc;

    LOGF_DEBUG("Moving Absolute Position: %g", value);

    if (isSimulation()) {
        simulatedPosition = value;
        return 0;
    }

    focuser_cmd[0] = 0;


    snprintf(focuser_cmd, 10, SET_NEW_POSITION, (int) value);

    if ((focuser_rc = sendCommand(focuser_cmd)) < 0)
        return focuser_rc;

    strncpy(focuser_cmd, FEED_GO, 5);
    if ((focuser_rc = sendCommand(focuser_cmd)) < 0)
        return focuser_rc;

    return 0;
}

bool StupidDogFocuser::SyncFocuser(uint32_t ticks) {
    char vl_tmp[10];
    int ret_read_tmp;

    if (isSimulation()) {
        currentPosition = ticks;
        return true;
    }

    snprintf(vl_tmp, 10, SET_CURRENT_POSITION, ticks);


    if ((ret_read_tmp = sendCommand(vl_tmp)) < 0)
        return false;

    return true;
}

bool StupidDogFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool StupidDogFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

void StupidDogFocuser::GetFocusParams() {
    int ret = -1;
    

    if ((ret = updateFocuserPosition(&currentPosition)) < 0) {
        FocusAbsPosNP.s = IPS_ALERT;
        LOGF_ERROR("Unknown error while reading Stupid Dog Focuser position: %d", ret);
        IDSetNumber(&FocusAbsPosNP, nullptr);
        return;
    }

    FocusAbsPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    if ((ret = updateFocuserTemperature(&currentTemperature)) < 0) {
        TemperatureNP.s = IPS_ALERT;
        LOG_ERROR("Unknown error while reading Stupid Dog Focuser temperature.");
        IDSetNumber(&TemperatureNP, nullptr);
        return;
    }

    TemperatureNP.s = IPS_OK;
    IDSetNumber(&TemperatureNP, nullptr);
}

IPState StupidDogFocuser::MoveAbsFocuser(uint32_t targetTicks) {
    int ret = -1;
    targetPos = targetTicks;

    if (targetTicks < FocusAbsPosN[0].min || targetTicks > FocusAbsPosN[0].max) {
        LOG_DEBUG("Error, requested position is out of range.");
        return IPS_ALERT;
    }

    if ((ret = updateFocuserPositionAbsolute(targetPos)) < 0) {
        LOGF_DEBUG("Read out of the absolute movement failed %3d", ret);
        return IPS_ALERT;
    }

    RemoveTimer(timerID);
    timerID = SetTimer(250);
    return IPS_BUSY;
}

IPState StupidDogFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks) {
    return MoveAbsFocuser(FocusAbsPosN[0].value + (ticks * (dir == FOCUS_INWARD ? -1 : 1)));
}

bool StupidDogFocuser::AbortFocuser() {
    LOG_DEBUG("Aborting focuser...");

    int nbytes_written;
    const char *buf = "\r";

    return tty_write(PortFD, buf, strlen(buf), &nbytes_written) == TTY_OK;
}

