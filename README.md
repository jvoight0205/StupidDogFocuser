# StupidDogFocuser

## Open-source electronic telescope focuser

The StupidDogFocuser is a work-in-progress prototype electronic focuser. It shall eventually comply with ASCOM and INDI and should operate on Linux, Windows, or Mac.

## Goals

The goal of this project is to develop an electronic focuser that is compact, free of airwires, capable of being mounted to a variety of telescope focusers, software/hardware controllable, inexpensive, easy to build at home, and only use components that are easy to acquire.

## Size and Shape

The PCB is a shield for an Arduino UNO R3. I hope to be able to etch and solder the board at home, but it should also be designed in such a way that one could order a commercially manufactured board. The KiCad design files will be included.

### DIY disclaimer
I'm not opposed to through-hole parts, but I personally find drilling boards to be the most difficult part of home manufacture of PCB, I'd rather keep the drill-file to a minimum. Consider that you're going to be drilling all of the Arduino header pins and all of the component pins. It's probably safest to order the boards from a board house.

As such, the KiCad PCBs will be done for _my_ convenience for now. That means tiny traces and silkscreen that I wouldn't dare do at home and drill holes galor and if I feel like throwing in a via collection that carries signals and looks like a moose, that's my perogative.

I mean, I probably won't, but just in case I do.

## Calibration and limit switches

Another goal is a limit switch and a reset routine. Upon hard reset (if there is such a thing), on power-up, it would store it's current position, rack the focuser IN until it touched the limit switch, reset everything such the limit switch is the low limit and the original position gets restored to it's whatever position it was before.

One way or another, I feel like I need some type of security such that my cameras never fall to the floor or I end up stripping a gear on the focuser.

That said, it hasn't been a problem _yet_.
## Power-Off Conditions

The calibration should survive a power-off, so we'll store the current positions and limits in EEPROM. I understand that we can't be sure that the focuser wasn't manually moved during a power outage, but I think we can trust the user to understand that if they move the focuser by hand during a power outage, they're going to have to recalibrate. In other words, if one has just just found focus at position 10,000 and the power cuts out at that moment, when the power comes back on, as long as nobody has dicked around with the focus knob, we should not-only still be in focus, we should still be at position 10,000 in the focuser hardware. Just keep in mind that if you hang a ton of gear off of a focuser without a strong reduction gear, your equipment will probably slide down. If that happens enough times, eventually your original calibration point will be long since lost and you may end up against the hard limits which will be 32-bit signed integers until somebody needs bigger.

As such, a sync command should be included so we can just focus by hand and tell it "that's the middle".

Ultimately, I want somebody to be able to have a neat, compact focuser for under $100 and their time and solder.


# As it is

As it is, we communicate at 115,200 baud. We depend on AccelStepper and a high performance encoder library, so I don't want to slow either of those down with slow serial performance. Our signal is only going a few feet, and unless you've got outrageous amounts of noise, 115,200 will be fine.

# Commands

Serial commands are as follows:

| Command | Description | Response        | Defaults | Comments |
| ---     | ---         | ---             | ---      | --- |
| <HA>	  | Halt        | HA!             | N/A      | as quickly as possible |
| <GE> 	  | Is enabled? | [T\|F]!          | True     | Motor Driver |
| <GR>     | Is reversed?| [T\|F]!          | False    | Motor direction |
| <GM>     | Microsteps  | [1-32]!         | _(1/)_1  | Denominator |
| <GH>     | High limit  | [-32768-32727]! | 32767    | Distance from initial |
| <GL>     | Low limit   | [-32768-32767]! | -32768   | 16 bit int |
| <GS>     | Speed       | [200-4000]!      | 1000     | Stepper motor dependent |
| <GT>     | Temperature | %f.2!             | N/A      | DHT11/22 precision  |
| <GP>     | Position    | [-32768-32727]! | 0        | Previous focus position |
| <IM>     | Moving?     | [T\|F]!          | F        | True during movement |
| <AM%d>   | Absolute Move | %d! (target) | N/A      | Target position|
| <RM%d>   | Relative Move | %d! (target) | N/A      | Target position |
| <RD[T\|F]>     | Reverse Motor | [T\|F]!       | N/A      | Response is new is reversed |
| <SY%d>   | Sync Motor | %d! (same as param) | N/A  | |
| <EN>     | Enable motor | EN!          | N/A| ||
| <DI>     | Disable motor| DI!          | N/A | |
| <SM[1-32]> | Set Microstepping 1/1 - 1/32 | %u! | 1 | New microstep mode |
| <SP%u>   | Set Speed 200-4000 | %d! | New speed | 1000 | Measure your motor to determine |
| <SH%d>   | Set high limit | %d! | 32767 | Response is new high |
| <SL%d>   | Set low limit  | %d! | -32768 | Response is new low limit |

Any error will trigger ERROR(optional message)#

