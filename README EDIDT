os:
  - linux
  - osx

sudo: false

dist: trusty

addons:
  apt:
    packages:
      - rpm

git:
  depth: 5

language: node_js

node_js:
  - 8.11.2

before_install:
  - npm i -g npm@6.0.1

script:
  - yarn gulp release

cache:
  directories:
  - node_modules

# Notifications are encrypted to betaflight/betaflight to avoid spam from forks
# Command: `travis encrypt "<secret>" --com -r betaflight/blackbox-log-viewer`
notifications:
  slack:
    rooms:
      - secure: lg53G5eeNucPlQos/RUA/PxkewvJwcWU7vxqMQJ8/Ba6VEcK4Yd6jGOhb837suafyZXR4HPqrOxBY8GNfcf8ZL258vLjfNy1164KL1ErUIUGAFKtevsV2BIX+G8LbEKkJDimKmzTZ4mZ4mnPY9fmRffTZY4Ioj+tj+DeESEUjo1AGWLWynRsBz3BondtE/7R8p+Ab3GPWv/JuL8Q4t3fJH8ZRSkdPC5rHuZKYge6EhrwCRlUzQP5iDC6Fw9ZOuSbsAcNLFh1ClaXDps9EPKSL5M7UMDYk1IgVrci0jcVo9afI7IZf5/SuR/Pn9BGSgSIV1+subarc0pew6PKG+Nz6dy9edhlTU/w7M1pi0/aOiUVEmo5nSsgJRwJg3lroonaAZpn9KWtNTRRMZDUK1wrge/QFKLydsrGrmxVo+QkjUf82MGQBAe5yDMfPZzkl502AwBp/ZnE6AT6Fj60A2sPsUgredK70f85Yv9r7FOX/mm+MU4MBAsDgl7LHHnC4Q9mH4kjnu2QBMf+1c7YxlOz8Y/a0b1+F72+e25wRrz/NWZfJLv6KVb7bjAFA9B8bSn9DKSauwloM/Bn2vQz64tmEJIEXm734nQx/fwii8RuaUJ/t8OJRiXiAF1IDBdKZNtE7jsMDBFNP/ct30Mm9Cqpad9xDFjRH5CgUYsMMLqphyU=
  webhooks:
    urls:
      - secure: qyk5ou1nFHaFJTeR5tUWqJdCKW6UPSrcedzhhcDpcT4seKtSycChzD6OJCJfdtI/9GbSw79H2E4chj49FzG9Qlilmq88TCIU61EKrxTmkUudXyxNKxBcUQ5S5N0y3GOABCUUxJpccDnLUHvD3MZ0ppQ43N4FdMygiq5yHwx1zXhBdnE6ZuwQGAwh/TPYOhPBbvB3W2xVXcQExWKCkhk8xGCA5vfvR0m82jkrwMSlfKLsWxmUNM6l5YDcHtCwE0F68WCtcTUiv6WVQ60SvVe1EVGiz0O43ee8K7Y/Ays23GSb839+N2NcAF0NY8BygPes3phbsw+zgJZB9PT3GmcY2A3U6n6SREtEy+7Zxc3m5ntI9FVrgepQqCnUHgu/sxV+eC356OXi1XIHK3Yv7ouh+gQTF3p3TVpMShesdW+zRzprt2x+PDFXv8ESu0DM6d7w0PU0YqcztON5fZQSvLytm+uXObIxDsT6oafdfR5J+hRHwEuBsPlMi7GaONQsMuxUC0nYBFq1VCHpVGLY2kbCxeqC35GmRH39Ei52U4Yr0fcgwa1Kh/2qTrjNtahrJhOcr+pE17XerPfWKCTfNARBI/AE274ykTNsKfhCQy6hETnw8xJm2GzARWmWK1Q3Ein0+C8Xs9LhQSrCpCGcKXsbsQZuqFY7gRrsbRGX9mzD7DI=
    on_success: always  # options: [always|never|change] default: always
    on_failure: always  # options: [always|never|change] default: always
    on_start: always     # options: [always|never|change] default: always
![Betaflight](docs/assets/images/bf_logo.png)

[![Latest version](https://img.shields.io/github/v/release/betaflight/betaflight)](https://github.com/betaflight/betaflight/releases) [![Build Status](https://dev.azure.com/Betaflight/Betaflight%20Nightlies/_apis/build/status/betaflight.betaflight?branchName=master)](https://dev.azure.com/Betaflight/Betaflight%20Nightlies/_build/latest?definitionId=8&branchName=master) [![Build Status](https://dev.azure.com/Betaflight/Betaflight%20Nightlies/_apis/build/status/betaflight.betaflight?branchName=4.2-maintenance)](https://dev.azure.com/Betaflight/Betaflight%20Nightlies/_build/latest?definitionId=8&branchName=4.2-maintenance) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

Betaflight is flight controller software (firmware) used to fly multi-rotor craft and fixed wing craft.

This fork differs from Baseflight and Cleanflight in that it focuses on flight performance, leading-edge feature additions, and wide target support.

## Events

| Date  | Event |
| - | - |


## News

### Requirements for the submission of new and updated targets

The following new requirements for pull requests adding new targets or modifying existing targets are put in place from now on:

1. No new F3 based targets will be accepted;

2. For any new target that is to be added, only a Unified Target config into https://github.com/betaflight/unified-targets/tree/master/configs/default needs to be submitted. See the [instructions](https://github.com/betaflight/betaflight/blob/master/docs/TargetMaintenance/CreatingAUnifiedTarget.md) for how to create a Unified Target configuration. If there is no Unified Target for the MCU type of the new target (see instructions above), then a 'legacy' format target definition into `src/main/target/` has to be submitted as well;

3. For changes to existing targets, the change needs to be applied to the Unified Target config in https://github.com/betaflight/unified-targets/tree/master/configs/default. If no Unified Target configuration for the target exists, a new Unified Target configuration will have to be created and submitted. If there is no Unified Target for the MCU type of the new target (see instructions above), then an update to the 'legacy' format target definition in `src/main/target/` has to be submitted alongside the update to the Unified Target configuration.


## Features

Betaflight has the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300, 600 and 1200), Multishot, and Oneshot (125 and 42) motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F7, F4 and F3 processors
* PWM, PPM, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CSRF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* OLED Displays - Display information on: Battery voltage/current/mAh, profile, rate profile, mode, version, sensors, etc
* In-flight manual PID tuning and rate adjustment
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* and MUCH, MUCH more.

## Installation & Documentation

See: https://github.com/betaflight/betaflight/wiki

## Support and Developers Channel

There's a dedicated Slack chat channel here:

https://slack.betaflight.com/

We also have a Facebook Group. Join us to get a place to talk about Betaflight, ask configuration questions, or just hang out with fellow pilots.

https://www.facebook.com/groups/betaflightgroup/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure Betaflight you should use the Betaflight-configurator GUI tool (Windows/OSX/Linux) which can be found here:

https://github.com/betaflight/betaflight-configurator/releases/latest

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

* implement a new feature in the firmware or in configurator (see [below](#Developers));
* documentation updates and corrections;
* How-To guides - received help? Help others!
* bug reporting & fixes;
* new feature ideas & suggestions;
* provide a new translation for configurator, or help us maintain the existing ones (see [below](#Translators)).

The best place to start is the Betaflight Slack (registration [here](https://slack.betaflight.com/)). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](docs/development/Development.md ) and [coding style definition](docs/development/CodingStyle.md).

TravisCI is used to run automatic builds

https://travis-ci.com/betaflight/betaflight

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 18 languages for Betaflight Configurator: Català, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:
- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the Betaflight Slack (registration [here](https://slack.betaflight.com/)), and join the '#team\_translation' channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Hardware Issues

Betaflight does not manufacture or distribute their own hardware. While we are collaborating with and supported by a number of manufacturers, we do not do any kind of hardware support.
If you encounter any hardware issues with your flight controller or another component, please contact the manufacturer or supplier of your hardware, or check RCGroups https://rcgroups.com/forums/showthread.php?t=2464844 to see if others with the same problem have found a solution.

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those whom have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight), and
* **Sambas** (for the original STM32F4 port).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....
