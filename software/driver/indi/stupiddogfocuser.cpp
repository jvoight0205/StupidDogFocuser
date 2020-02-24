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

#define currentPosition         FocusAbsPosN[0].value
#define currentTemperature      TemperatureN[0].value
#define currentMinPosition      MinMaxPositionN[0].value
#define currentMaxPosition      MinMaxPositionN[1].value

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
            FOCUSER_CAN_REVERSE |
            FOCUSER_HAS_BACKLASH);
    setSupportedConnections(CONNECTION_SERIAL);
    serialConnection = new Connection::Serial(this);
    //serialConnection->registerHandshake([&]() { return Handshake(); });
    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
  
    LOG_INFO("Initialized.");

}

bool StupidDogFocuser::initProperties() {
    INDI::Focuser::initProperties();

    /* Focuser temperature */
    IUFillNumber(&TemperatureN[0], "TEMPERATURE", "Celsius", "%6.2f", -60., 190., 0., 10000.);
    IUFillNumberVector(&TemperatureNP, TemperatureN, 1, getDeviceName(), "FOCUS_TEMPERATURE", "Temperature",
            MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    /* Stupid Dog Focuser should stay within these limits */
    IUFillNumber(&MinMaxPositionN[0], "MINPOS", "Minimum Tick", "%d", -32768, 32767, 0, -32768);
    IUFillNumber(&MinMaxPositionN[1], "MAXPOS", "Maximum Tick", "%d", -32768, 32767, 0, 32767);
    // Place on Settings Tab
    IUFillNumberVector(&MinMaxPositionNP, MinMaxPositionN, 2, getDeviceName(), "FOCUS_MINMAXPOSITION", "Extrema",
            SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    // Reversed
    IUFillSwitch(&Reversed[0], "FORWARD", "Forward", ISS_OFF);
    IUFillSwitch(&Reversed[0], "REVERSE", "Reverse", ISS_ON);

    IUFillSwitchVector(&ReversedSP, Reversed, 2, getDeviceName(), "DIRECTION", "Motor Direction",
            SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Enabled
    IUFillSwitch(&Enabled[0], "ENABLE", "Enable", ISS_ON);
    IUFillSwitch(&Enabled[1], "DISABLE", "Disable", ISS_OFF);
    IUFillSwitchVector(&EnabledSP, Enabled, 2, getDeviceName(), "ENABLE", "Motor Power",
            SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Microstep
    IUFillNumber(&MicrostepN[0], "MICROSTEP", "Microstep reciprocal", "%u", 1, 32, 0, 1);
    // Place on Settings Tab
    IUFillNumberVector(&MicrostepNP, MicrostepN, 2, getDeviceName(), "FOCUS_MICROSTEP", "Microstep 1/n",
            SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    // Speed
    IUFillNumber(&SpeedN[0], "SPEED", "Speed", "%u", 200, 3000, 0, 1000);
    // Place on Settings Tab
    IUFillNumberVector(&SpeedNP, SpeedN, 1, getDeviceName(), "SPEED", "Speed",
            SETTINGS_TAB, IP_RW, 0, IPS_IDLE);


    SpeedN[0].value = 250;
    /* Relative and absolute movemennt */

    FocusAbsPosN[0].min = -32768;
    FocusAbsPosN[0].max = 32767;
    FocusAbsPosN[0].value = 0;
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
        defineNumber(&MicrostepNP);
        defineNumber(&SpeedNP);
        defineSwitch(&EnabledSP);
        defineSwitch(&ReversedSP);

        GetFocusParams();

        LOG_DEBUG("StupidDogFocuser parameters readout complete, focuser ready for use.");
    } else {
        deleteProperty(TemperatureNP.name);
        deleteProperty(MinMaxPositionNP.name);
        deleteProperty(MicrostepNP.name);
        deleteProperty(SpeedNP.name);
        deleteProperty(EnabledSP.name);
        deleteProperty(ReversedSP.name);

    }

    return true;
}

bool StupidDogFocuser::Handshake() {
    char firmware[MAX_BUFFER];
    char reply[MAX_BUFFER];

    if (isSimulation()) {
        timerID = SetTimer(POLLMS);
        LOG_INFO("Stupid Dog Focuser Simulator: online. Getting focus parameters...");
        FocusAbsPosN[0].value = simulatedPosition;
        //updateFocuserFirmware(firmware, reply);
        return true;
    }
    tcflush(PortFD, TCIOFLUSH);

    if ((updateFocuserFirmware(firmware, reply)) < 0) {
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

    //tcflush(PortFD, TCIOFLUSH);
    //tty_nread_section(PortFD, responseBuffer, strlen(reply) - 1, '>', .5, &nbytes_read); // clear buffer
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
        tcflush(PortFD, TCIOFLUSH);
        return false;
    }

    responseBuffer[strlen(responseBuffer) - 1] = '\0';
    LOGF_DEBUG("Response from command ------------------- '%s'", responseBuffer);
    strncpy(reply, responseBuffer, MAX_BUFFER);
    return nbytes_read - 1;
}
//
//int StupidDogFocuser::readResponse(char *res) {
//    int nbytes_read = 0, rc = -1, res_len = strlen(res);
//    LOG_DEBUG("* * * * * * * * * * * * * * * * * * Reading serial stream.");
//    if ((rc = tty_nread_section(PortFD, res, res_len, '>', ML_TIMEOUT, &nbytes_read)) != TTY_OK) {
//        char errstr[MAXRBUF] = {0};
//        tty_error_msg(rc, errstr, MAXRBUF);
//        LOGF_ERROR("* * * * * * * * * * * * * * * * * *Serial read error: %s. \"%s\"", errstr, res);
//        return false;
//    }
//
//    int len = strlen(res);
//    res[len - 1] = 0;
//    LOGF_DEBUG("* * * * * * * * * * * * * * * * * *RES <%s>", res);
//
//    tcflush(PortFD, TCIOFLUSH);
//
//    return nbytes_read - 1;
//}

int StupidDogFocuser::updateFocuserPosition(double *value) {
    int temp;

    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;

    LOG_DEBUG("Querying Position...");

    if (isSimulation()) {
        *value = simulatedPosition;
        return 0;
    }

    strncpy(focuser_cmd, GET_POSITION, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;

    if (sscanf(focuser_reply, SIGNED_RESPONSE, &temp) < 1)
        return -1;

    *value = (double) temp;

    LOGF_DEBUG("Position: %g", *value);

    return 0;
}

int StupidDogFocuser::updateFocuserTemperature(double *value) {
    LOGF_DEBUG("Update Temperature: %g", value);

    int temp;
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;

    strncpy(focuser_cmd, GET_TEMPERATURE, MAX_BUFFER);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;

    if (isSimulation())
        snprintf(focuser_cmd, FOCUSER_MAX_CMD, "-99.9");

    if (sscanf(focuser_reply, SIGNED_RESPONSE, &temp) < 1)
        return -1;

    *value = (double) temp;

    return 0;
}

int StupidDogFocuser::updateFocuserFirmware(char *_focuser_cmd, char *_focuser_reply) {
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

int StupidDogFocuser::updateFocuserPositionAbsolute(double value) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int focuser_rc;
    int newPosition;

    LOGF_DEBUG("Moving Absolute Position: %d", (int) value);

    if (isSimulation()) {
        simulatedPosition = value;
        return 0;
    }

    snprintf(focuser_cmd, MAX_BUFFER, ABSOLUTE_MOVE, (int) value);

    if ((focuser_rc = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return focuser_rc;


    if (sscanf(focuser_reply, SIGNED_RESPONSE, &newPosition) < 1)
        return -1;

    currentPosition = newPosition;
    value = (double) newPosition;

    return 0;
}

bool StupidDogFocuser::SyncFocuser(uint32_t ticks) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int ret_read_tmp;

    if (isSimulation()) {
        currentPosition = ticks;
        return true;
    }

    snprintf(focuser_cmd, MAX_BUFFER, SYNC_MOTOR, ticks);


    if ((ret_read_tmp = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return false;


    if (sscanf(focuser_reply, SIGNED_RESPONSE, &ticks) < 1)
        return false;

    currentPosition = ticks;
    return true;
}

bool StupidDogFocuser::SetFocuserSpeed(int speed) {
    char focuser_cmd[MAX_BUFFER];
    char focuser_reply[MAX_BUFFER];
    int ret_read_tmp;

    if (isSimulation()) {
        currentPosition = speed;
        return true;
    }

    snprintf(focuser_cmd, MAX_BUFFER, SET_SPEED, speed*19);


    if ((ret_read_tmp = sendCommand(focuser_cmd, focuser_reply)) < 0)
        return false;


    if (sscanf(focuser_reply, UNSIGNED_RESPONSE, &ret_read_tmp) < 1)
        return false;

    speed = ret_read_tmp / 19;
    return true;
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
    stupidDogFocuser->ISNewText(dev, name, texts, names, n);
}

bool StupidDogFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool StupidDogFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

void StupidDogFocuser::GetFocusParams() {
    int ret = -1;
    LOG_INFO("Getting Focus Params.");

    if ((ret = updateFocuserPosition(&currentPosition)) < 0) {
        FocusAbsPosNP.s = IPS_ALERT;
        LOGF_ERROR("* * * * * * * * * * * * * * * * * *Unknown error while reading Stupid Dog Focuser position: %d", ret);
        IDSetNumber(&FocusAbsPosNP, nullptr);
        return;
    }

    FocusAbsPosNP.s = IPS_OK;
    IDSetNumber(&FocusAbsPosNP, nullptr);

    if ((ret = updateFocuserTemperature(&currentTemperature)) < 0) {
        TemperatureNP.s = IPS_ALERT;
        LOG_ERROR("* * * * * * * * * * * * * * * * * *Unknown error while reading Stupid Dog Focuser temperature.");
        IDSetNumber(&TemperatureNP, nullptr);
        return;
    }

    TemperatureNP.s = IPS_OK;
    IDSetNumber(&TemperatureNP, nullptr);
    
    // Speed
    
    // Microstep
    
    // Is Reversed
}

IPState StupidDogFocuser::MoveAbsFocuser(uint32_t targetTicks) {
    int ret = -1;
    targetPos = targetTicks;

    if (targetTicks < FocusAbsPosN[0].min || targetTicks > FocusAbsPosN[0].max) {
        LOG_DEBUG("* * * * * * * * * * * * * * * * * *Error, requested position is out of range.");
        return IPS_ALERT;
    }

    if ((ret = updateFocuserPositionAbsolute(targetPos)) < 0) {
        LOGF_DEBUG("* * * * * * * * * * * * * * * * * *Read out of the absolute movement failed %3d", ret);
        return IPS_ALERT;
    }


    return IPS_OK;
}

IPState StupidDogFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks) {
    return MoveAbsFocuser(FocusAbsPosN[0].value + (ticks * (dir == FOCUS_INWARD ? -1 : 1)));
}

bool StupidDogFocuser::AbortFocuser() {
    LOG_DEBUG("* * * * * * * * * * * * * * * * * *Aborting focuser...");

    int nbytes_written;
    const char *buf = "<HA>\r";

    return tty_write(PortFD, buf, strlen(buf), &nbytes_written) == TTY_OK;
}

