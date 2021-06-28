# Failsafe

There are two types of failsafe:

1. Receiver based failsafe
2. Flight controller based failsafe

Receiver based failsafe is where you, from your transmitter and receiver, configure channels to output desired signals if your receiver detects signal loss and goes to the __failsafe mode__. The idea is that you set throttle and other controls so the aircraft descends in a controlled manner.  See your receiver's documentation for this method.

Flight controller based failsafe is where the flight controller attempts to detect signal loss and/or the __failsafe mode__ of your receiver and upon detection goes to __failsafe stage 1__. The idea is that the flight controller starts using __fallback settings__ for all controls, which are set by you, using the CLI command `rxfail` (see [rxfail](Rx.md#rx-loss-configuration) section in rx documentation) or the cleanflight-configurator GUI.

It is possible to use both types at the same time, which may be desirable.  Flight controller failsafe can even help if your receiver signal wires come loose, get damaged or your receiver malfunctions in a way the receiver itself cannot detect.

Alternatively you may configure a transmitter switch to activate failsafe mode. This is useful for fieldtesting the failsafe system and as a **_`PANIC`_** switch when you lose orientation.

## Flight controller failsafe system 

This system has two stages.

__Stage 1__ is entered when __a flightchannel__ has an __*invalid pulse length*__, the receiver reports __*failsafe mode*__ or there is __*no signal*__ from the receiver. Fallback settings are applied to __*all channels*__ and a short amount of time is provided to allow for recovery.

__Note:__ Prior to entering __stage 1__, fallback settings are also applied to __*individual AUX channels*__ that have invalid pulses.

__Stage 1__ can also directly be activated when a transmitter switch that is configured to control the failsafe mode is switched ON and `failsafe_switch_mode` is set to `STAGE1`. Stage 1 will be aborted if the switch is moved to the OFF position before Stage 2 is engaged (see next).

__Stage 2__ is entered when your craft is __armed__ and __stage 1__ persists longer then the configured guard time (`failsafe_delay`). All channels will remain at the applied fallback setting unless overruled by the chosen stage 2 procedure (`failsafe_procedure`).

__Stage 2__ is not activated until 5 seconds after the flight controller boots up.  This is to prevent unwanted activation, as in the case of TX/RX gear with long bind procedures, before the RX sends out valid data.

__Stage 2__ can also directly be activated when a transmitter switch that is configured to control the failsafe mode is switched ON and `failsafe_switch_mode` is set to `STAGE2`.

__Stage 2__ will be aborted when it was due to:

* a lost RC signal and the RC signal has recovered.
* a transmitter failsafe switch was set to ON position and the switch is set to OFF position (and `failsafe_switch_mode` is _not_ set to `KILL`).

Note that:
* At the end of the stage 2 procedure, the flight controller will be disarmed and re-arming will be locked until the signal from the receiver is restored for specific amount of time depending on the procedure (see below) AND the arming switch is in the OFF position (when an arm switch is in use).

* Prior to starting a stage 2 intervention it is checked if the throttle position was below `min_throttle` level for the last `failsafe_throttle_low_delay` seconds. If it was, the craft is assumed to be on the ground and is only disarmed. It may be re-armed without a power cycle.     

Some notes about **SAFETY**:
* The failsafe system will be activated regardless of current throttle position. So when the failsafe intervention is aborted (RC signal restored/failsafe switch set to OFF) the current stick position will direct the craft !
* The craft may already be on the ground with motors stopped and that motors and props could spin again - the software does not currently detect if the craft is on the ground.  Take care when using `MOTOR_STOP` feature. **Props will spin up without warning**, when armed with `MOTOR_STOP` feature ON (props are not spinning) **_and_** failsafe is activated !

## Configuration

When configuring the flight controller failsafe, use the following steps:

1.  Configure your receiver to do one of the following:

* Upon signal loss, send no signal/pulses over the channels
* Send an invalid signal over the channels (for example, send values lower than `rx_min_usec`)

and

* Ensure your receiver does not send out channel data that would cause a disarm by switch or sticks to be registered by the FC. This is especially important for those using a switch to arm.

See your receiver's documentation for direction on how to accomplish one of these.

* Configure one of the transmitter switches to activate the failsafe mode.

2.  Set `failsafe_off_delay` to an appropriate value based on how high you fly

3.  Set `failsafe_throttle` to a value that allows the aircraft to descend at approximately one meter per second (default is 1000 which should be throttle off).




These are the basic steps for flight controller failsafe configuration; see Failsafe Settings below for additional settings that may be changed.

## Failsafe Settings

Failsafe delays are configured in 0.1 second steps.

1 step = 0.1sec

1 second = 10 steps

### `failsafe_delay`

Guard time for failsafe activation after signal lost.  This is the amount of time the flight controller waits to see if it begins receiving a valid signal again before activating failsafe.

### `failsafe_off_delay`

Delay after failsafe activates before motors finally turn off.  This is the amount of time 'failsafe_throttle' is active.  If you fly at higher altitudes you may need more time to descend safely.

### `failsafe_throttle`

Throttle level used for landing.  Specify a value that causes the aircraft to descend at about 1M/sec. Default is set to 1000 which should correspond to throttle off.

### `failsafe_switch_mode`

Configure the RC switched failsafe action. It can be one of:
* `STAGE1` - activates Stage 1 failsafe. RC controls are applied as configured for Stage 1 and the `failsafe_delay` guard time will have to elapse before Stage 2 is activated. This is useful if you want to simulate with a switch the exact signal loss failsafe behavior.
* `STAGE2` - skips Stage 1 and activates the Stage 2 procedure immediately (see `failsafe_procedure`). Useful if you want to assign instant auto-landing to a switch.
* `KILL` - disarms instantly (your craft will crash). Re-arming is locked for 1 second AND until the arming switch (if used) is moved to the OFF position. Similar effect can be achieved by:
  * setting `failsafe_switch_mode` to `STAGE2` and `failsafe_procedure` to `DROP`. The difference is that `DROP` locks re-arming for 3 seconds instead of 1.
  * setting `failsafe_switch_mode` to `STAGE2`, `failsafe_procedure` to `AUTO-LAND`, setting `failsafe_throttle` to 1000 and `failsafe_off_delay` to 0 (basically initiates an auto-landing but cuts it short immediately). This is not preferred method, since the reaction is slower and re-arming will be locked for 30 seconds.
  * using arm switch. This does not introduce re-arming locking.

### `failsafe_throttle_low_delay`

Time throttle level must have been below 'min_throttle' to _only disarm_ instead of _full failsafe procedure_.

Use standard RX μs values.  See [Rx documentation](Rx.md).

### `failsafe_procedure`

* `DROP`: Just kill the motors and disarm (crash the craft). Re-arming is locked until RC link is available for at least 3 seconds and the arm switch (if used) is in the OFF position.
* `AUTO-LAND`: Enable an auto-level mode, center the flight sticks and set the throttle to a predefined value (`failsafe_throttle`) for a predefined time (`failsafe_off_delay`). This should allow the craft to come to a safer landing. Re-arming is locked until RC link is available for at least 30 seconds and the arm switch (if used) is in the OFF position.

### `rx_min_usec`

The lowest channel value considered valid.  e.g. PWM/PPM pulse length 

### `rx_max_usec`

The highest channel value considered valid.  e.g. PWM/PPM pulse length 

The `rx_min_usec` and `rx_max_usec` settings helps detect when your RX stops sending any data, enters failsafe mode or when the RX looses signal.

With a Graupner GR-24 configured for PWM output with failsafe on channels 1-4 set to OFF in the receiver settings then this setting, at its default value, will allow failsafe to be activated.

## Testing

**Bench test the failsafe system before flying - _remove props while doing so_.**

1. Arm the craft.
1. Turn off transmitter or unplug RX.
1. Observe motors spin at configured throttle setting for configured duration.
1. Observe motors turn off after configured duration.
1. Ensure that when you turn on your TX again or reconnect the RX that you cannot re-arm once the motors have stopped.
1. Power cycle the FC.
1. Arm the craft.
1. Turn off transmitter or unplug RX.
1. Observe motors spin at configured throttle setting for configured duration.
1. Turn on TX or reconnect RX.
1. Ensure that your switch positions don't now cause the craft to disarm (otherwise it would fall out of the sky on regained signal).
1. Observe that normal flight behavior is resumed.
1. Disarm.
 
**Field test the failsafe system.**

1. Perform bench testing first!
1. On a calm day go to an unpopulated area away from buildings or test indoors in a safe controlled environment - e.g. inside a big net.
1. Arm the craft.
1. Hover over something soft (long grass, ferns, heather, foam, etc.).
1. Descend the craft and observe throttle position and record throttle value from your TX channel monitor.  Ideally 1500 should be hover. So your value should be less than 1500.
1. Stop, disarm.
1. Set failsafe throttle to the recorded value.
1. Arm, hover over something soft again.
1. Turn off TX (!)
1. Observe craft descends and motors continue to spin for the configured duration.
1. Observe FC disarms after the configured duration.
1. Remove flight battery.

If craft descends too quickly then increase failsafe throttle setting.

Ensure that the duration is long enough for your craft to land at the altitudes you normally fly at.

Using a configured transmitter switch to activate failsafe mode, instead of switching off your TX, is good primary testing method in addition to the above procedure.
