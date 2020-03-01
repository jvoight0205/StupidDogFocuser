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

#define HALT "HA"
#define IS_ENABLED "GE"
#define IS_REVERSED "GR"
#define GET_MICROSTEP "GM"
#define GET_HIGH_LIMIT "GH"
#define GET_SPEED "GS"
#define GET_TEMPERATURE "GT"
#define GET_POSITION "GP"
#define IS_MOVING "GV"
#define ABSOLUTE_MOVE "AM%ld"
#define RELATIVE_MOVE "RM%ld"
#define REVERSE_DIR "RD"
#define FORWARD_DIR "FD"
#define SYNC_MOTOR "SY%ld"
#define ENABLE_MOTOR "EN"
#define DISABLE_MOTOR "DI"
#define SET_MICROSTEP "SM%u"
#define SET_SPEED "SP%u"
#define SET_HIGH_LIMIT "SH%ld"
#define TRUE_RESPONSE "T"
#define FALSE_RESPONSE "F"
#define SIGNED_RESPONSE "%d"
#define UNSIGNED_RESPONSE "%u"
#define LONG_RESPONSE "%ld"
#define FLOAT_RESPONSE "%f"
#define GET_VERSION "VE"


// Version should match hardware response to make sure our protocol works.
#define VERSION ".9"

#define MAX_BUFFER 64

class StupidDogFocuser : public INDI::Focuser {
public:
    StupidDogFocuser();
    virtual ~StupidDogFocuser() override = default;

    typedef enum {
        M1, M2, M4, M8, M16, M32
    } MicrostepMode;


    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    bool SetFocuserMicrostep(int _microstep);

protected:
    virtual bool Handshake() override;
    const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual bool AbortFocuser() override;
    virtual bool SyncFocuser(uint32_t ticks) override;
    virtual bool SetFocuserSpeed(int speed) override;
    virtual void TimerHit() override;



private:
    int sendCommand(char *cmd, char *res);
    void GetFocusParams();
    int readFocuserPosition();
    int readFocuserTemperature();
    int readFocuserSpeed();
    int readFocuserMicrostep();
    int readFocuserFirmware(char *_focuser_cmd, char *_focuser_reply);
    int sendFocuserPositionAbsolute(uint32_t newTarget);
    int sendFocuserSyncPosition(const double *newPosition);
    int sendFocuserSpeed(int *speed);
    int sendFocuserMicrostep(int *microstep);
    int readFocuserMoving();
    int sendFocuserEnable();
    int sendFocuserDisable();
    int sendFocuserHighLimit(double *highLimit);



    int timerID{ -1};
    double targetPos{ 0};
    bool simulatedMoving{false};
    int8_t dir{1};
    uint8_t microstep{1};
    uint8_t speed{255};
    double simulatedTemperature{ 0};
    double simulatedPosition{ 0};
    char commandBuffer[MAX_BUFFER];
    char responseBuffer[MAX_BUFFER];

    ILight MovingL[1];
    ILightVectorProperty MovingLP;

    INumber TemperatureN[1];
    INumberVectorProperty TemperatureNP;

    // Step mode
    ISwitch MicrostepModeS[6];
    ISwitchVectorProperty MicrostepModeSP;

    ISwitch Enabled[2];
    ISwitchVectorProperty EnabledSP;

    // Update status and perform maintenance every 2 seconds 

    // Time for the arduino to boot up.
    static const uint8_t ML_TIMEOUT{ 5};
};
