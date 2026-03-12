# Remote Node Firmware

STM32 firmware for the remote temperature measurement node in the temperature logger system.

This firmware runs on an STM32F103-based remote node and is responsible for:

- measuring local temperatures
- maintaining the latest sampled values
- receiving RF requests from the gateway over nRF24L01+
- returning formatted measurement payloads to the gateway

## Role in the system

The remote node acts as the wireless sensor endpoint in the distributed measurement system.

Typical flow:

- Gateway sends RF request `GTMP`
- Remote node prepares a `TMP!` response payload
- Gateway receives the response and returns formatted data to the host over UART

## Repository context

This repository contains the actual STM32CubeIDE firmware project for the remote node.

The broader system documentation is maintained separately in:

- **stm32-nrf24-temperature-logger**: `https://github.com/honkajan/stm32-nrf24-temperature-logger`

Related repositories:

- **uartctl** (host-side Python CLI tool): `https://github.com/honkajan/uartctl`
- **gateway-fw** (gateway firmware): `https://github.com/honkajan/gateway-fw`

## Project layout

This repository contains the original STM32CubeIDE project.

The CubeIDE project currently retains its historical working name:

- `bathroom_remote_ntc`

That name reflects the project's earlier application-specific bring-up phase.

## Development environment

- STM32CubeIDE project
- STM32F103C8T6 target
- STM32 HAL / CMSIS drivers included in the repository
- nRF24L01+ radio module
- local temperature measurement using ADC-based sensing

## Opening the project

Open the repository as an STM32CubeIDE project.

Key project files include:

- `.project`
- `.cproject`
- `.mxproject`
- `bathroom_remote_ntc.ioc`

The `.ioc` file can be opened with STM32CubeMX integration inside STM32CubeIDE if peripheral configuration inspection or regeneration is needed.

## Notes on build artifacts

Build output directories and local IDE/debug launch files are intentionally not tracked in Git.

Examples of excluded local/generated content:

- `Debug/`
- `.settings/`
- `*.launch`

## Current focus

The firmware has evolved from local temperature measurement and UART-oriented bring-up into a wireless remote sensing node with support for:

- periodic local sampling
- RF request/response handling
- gateway-triggered measurement reporting
- shared radio protocol development with the gateway node

## Status

This is an actively developed embedded firmware project and may continue to evolve as the overall system is refined.

For system-level architecture, protocol documentation, and diagrams, see the main system repository.

## License

See the repository license file.