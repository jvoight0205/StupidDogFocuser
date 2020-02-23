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

#pragma once

#include "indifocuser.h"
#include "connectionplugins/connectionserial.h"

#define HALT "HA#"
#define IS_ENABLED "GE#"
#define IS_REVERSED "GR#"
#define GET_MICROSTEP "GM#"
#define GET_HIGH_LIMIT "GH#"
#define GET_LOW_LIMIT "GL#"
#define GET_SPEED "GS#"
#define GET_TEMPERATURE "GT#"
#define GET_POSITION "GP#"
#define IS_MOVING "GV#"
#define ABSOLUTE_MOVE "AM%d# "
#define RELATIVE_MOVE "RM%d# "
#define REVERSE_DIR "RD#"
#define FORWARD_DIR "FD#"
#define SYNC_MOTOR "SY%d# "
#define ENABLE_MOTOR "EN#"
#define DISABLE_MOTOR "DI#"
#define SET_MICROSTEP "SM%u# "
#define SET_SPEED "SP%u#"
#define SET_HIGH_LIMIT "SH%d# "
#define SET_LOW_LIMIT "SL%d# "
#define SIGNED_RESPONSE "%d# "
#define UNSIGNED_RESPONSE "%u# "
#define TRUE_RESPONSE "T#"
#define FALSE_RESPONSE "F#"
#define FLOAT_RESPONSE "%f# "
#define GET_VERSION "VE#"


// Version should match hardware response to make sure our protocol works.
#define VERSION .9

class StupidDogFocuser : public INDI::Focuser {
public:
    StupidDogFocuser();
    virtual ~StupidDogFocuser() override = default;

    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

protected:
    virtual bool Handshake() override;
    const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual bool AbortFocuser() override;
    virtual bool SyncFocuser(uint32_t ticks) override;

private:
    int sendCommand(const char *cmd);
    int readResponse(char *_resp);
    void GetFocusParams();

    int updateFocuserPosition(double *value);
    int updateFocuserTemperature(double *value);
    int updateFocuserFirmware(char *_focuser_cmd);
    int updateFocuserPositionAbsolute(double value);
    int updateFocuserSetPosition(const double *value);

    int timerID{ -1};
    double targetPos{ 0};
    double simulatedTemperature{ 0};
    double simulatedPosition{ 0};

    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

    INumber MinMaxPositionN[2];
    INumberVectorProperty MinMaxPositionNP;
    
    INumber SpeedN[1];
    INumberVectorProperty SpeedNP;

    INumber MicrostepN[1];
    INumberVectorProperty MicrostepNP;

    ISwitch Reversed[2];
    ISwitchVectorProperty ReversedSP;

    ISwitch Enabled[2];
    ISwitchVectorProperty EnabledSP;

    // MyFocuserPro2 Buffer
    static const uint8_t ML_RES{ 32};

    // MyFocuserPro2 Delimeter
    static const char ML_DEL{ 0xA};

    // MyFocuserPro2 Timeout
    static const uint8_t ML_TIMEOUT{ 3 };

};
