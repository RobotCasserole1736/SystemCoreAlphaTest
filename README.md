# SystemCoreAlphaTest

Codebase for the 2025 SystemCore Alpha test. This codebase is targeted at our Practice Bot which, while fairly barebones, still ~~is very cursed~~ has great test coverage:

1) a mix of Rev and Kraken motors on every swerve drive module
2) Pulse-width absolute encoders on the azimuth sensors (horray COVID supply chain constraints that turned into technical inertia)
3) A whole website that the robot code hosts
4) A javscript client talking to ntcore on the new SystemCore
5) A completely custom swerve control implementation in python
6) Repulsor-field on-the-fly navigation
7) A vertically-mounted RIO


# Notes

At least as of 7/25 - 

1) CTRE has done a one-off release of wheels - https://ctre.download/files/Phoenix%206%2025.90.0a1%20Python%20Wheels.zip - need to be downloaded and installed
