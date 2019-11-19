# Ragnarok-2020
Repository containing all software and electronics for the 2020 International CanSat Competition Team Ragnarok. Software is based around the STM32 family of ARM microcontrollers. All electronics are designed to meet the mission requirements described [here](http://cansatcompetition.com/docs/CanSat_Mission_Guide_2020.pdf).

__All mission reviews and documentation are currently hosted separately__.

### Software
#### Pat needs to fill this in
Development is being done in VSCode using the PlatformIO extension.
Requirements/configuration: _something about python, pip, pyserial, etc..._
`sudo apt-get install python3 -m pip`
`pip3 install pyserial`

Software team:
- Patrick Smith (lead)
- Noah Schwalb

For more information about software development contact __PATS INFO HERE__

### Electrical
Electrical development is completed using [KiCad](http://www.kicad-pcb.org).

#### Documentation
The directory `/hardware/documentation/` contains important datasheets and application notes for the microcontrollers and components to be used within this project. These documents contain application specific information, and they should contain answers to any questions that may be had about these components.

#### Trade studies
The directory `/hardware/trade-studies/` contains datasheets for each of the sensors considered in this project. Detailed trade-studies have been completed and are hosted outside of this repository.

#### Libraries
The directory `/hardware/libraries/` contains the part libraries to be used within each of the PCBs. These libraries will include schematic symbols and physical footprints for each component. Libraries are first drawn from the [KiCad](http://kicad-pcb.org/libraries/download/) standard library and [Digikey-KiCad](https://www.digikey.com/en/resources/design-tools/kicad) standard library. If a symbol or footprint is missing from this library, [SnapEDA](https://www.snapeda.com) will be the next resource used. If the component cannot be found in any of the previous sources, a custom footprint/symbol will be created and housed within a `ragnarok_custom_components` library.

Electrical team:
- Alex Spaulding (lead)
- Shelby Westrich
- Brennan Haralson
- Shelby Tull

For more information about hardware development contact Alex Spaulding at ars0043@uah.edu
