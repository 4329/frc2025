# Notes

Author: Hayden Schroeder

## Overview

Need to review code to refresh myself with FRC command and subsystem framework. Also need to determine if it would be better to reuse sections of 2025 code or begin from scratch.

## FRC Command & Subsystem Framework

### `Robot.java`

- Contains main entrypoint for code. Methods such as `robotInit()` and `teleopInit()`. Also calls same name methods in `RobotContainer.java`
- `Logger` is initialized and configured here. This is new to me.
- `robotPeriodic()` runs the command scheduler.
- `autonomousInit()` selects a singular autonomous command to run.

### `RobotContainer.java`

- Initialize all subsystems, controllers, button bindings,
- What is the `CommandLoginator`?
- Contains autonomous builder
- Also contains methods such as `robotInit()` and `teleopInit()`, but these are called from Robot.java. Kinda goofy

### `Constants.java`

- Sets hardcoded constants as well as those in the config file (stored locally on robot linux home directory)
- Also uses `utilites.HorrayConfig.java` to pull hardcoded config values from `utilities.Config.java`

### `limlihSettings`

Constants for limelight

### `utilities`

Utility functions.

### `subsystems.LoggingSubsystem`

- Looks like this generates a separate log for each subsystem
- Each class inherites the `LoggedSubsystem` interface. This allows each system to have a set of `LoggableInputs`. My guess is the loggable inputs are then just logged to each subsystem log file.
- Works with `ulilities.CommandLoginator` somehow in order to log (more?)

### `model`

Contains data models for various subsystems for logging purposes.

### The Other Stuff

There's a bunch of different subsytems and commands in here. What's actually being used?

## Reuse 2025?

- Anything non-dependent on hardware can be reused. The rest is kind of up to the hardware. As long as the system remains the same it will be easily reusable. For example, a swap of motor types on swerve drive shouldn't be too much of an issue.
- Definitely some weirdly named stuff, but good for the most part
- How much does WPILib generate for you?
- Should I make a "clean" version of the 2025 codebase for reference? With the amount of commands and subsystems I'm seeing, my guess is there is quite a bit of deprecated code in here.

### YASS

This software suite looks pretty awesome. With the new team, this could be a great way to get everyone started.

- YAGSL (swerve) looks awesome
- YAMS (mechanism system) is a great solution for generating code which is easily simulated
- YAMG (mechanism generator) used if you don't want to use YAMS. Generates customizable code for mechanisms. Not easily simulated though.
- YALL (limelight) looks awesome

### What to Reuse

I think the move will be to reuse our unique core software pieces, and then use YAGSL, YAMS, and YALL for the rest. Since we have almost an entirely new software team, I think this approach is the best fit.

- [ ] Logger
- [ ] Certain constants
- [ ] Config
- [ ] Certain utility functions

### Message to Joey and Cameron

After reviewing the 2025 codebase and YASS, I think the best approach for 2026 is to reuse our unique core software pieces (logger, config, certain utility functions) and then use YAGSL, YAMS, and YALL for the rest. Since we have almost an entirely new software team, I think this approach will be the easiest.

If you have no objections, I'd like to assist by generating a new project using WPILib and integrating our software into it. This way you will have a clean codebase to build on for the future. I don't plan on writing any new code for the robot (and I hope to never need to ), but I can remove a lot of the deprecated code from 2025 for this new 2026 codebase.

## TODO

- [x] Generate a default WPILib project to understand how much it writes for you.
- [ ] Review and understand logger
- [ ] Review and understand config system
- [x] Understand simulations
