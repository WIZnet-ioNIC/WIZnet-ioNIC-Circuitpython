# Getting Started

These sections will guide you through a series of steps from configuring development environment to running W55RP20 examples with CircuitPython.

- [**Hardware Requirements**](#hardware_requirements)
- [**Development Environment Configuration**](#development_environment_configuration)
    - [**Installing CircuitPython**](#installing_circuitpython)
    - [**Setup Libraries**](#setup_libraries)
- [**Example Structure**](#example_structure)
- [**Example Testing**](#example_testing)



<a name="hardware_requirements"></a>

## Hardware Requirements

- [**W55RP20-EVB-Pico**][link-w55rp20-evb-pico]
- **Desktop** or **Laptop**
- **USB C Type Cable**



<a name="development_environment_configuration"></a>

## Development Environment Configuration

To test the examples, the development environment must be configured to use W55RP20-EVB-Pico.

The examples were tested by configuring the development environment for **Windows**. Please refer to the below and configure accordingly.



<a name="installing_circuitpython"></a>
### Installing CircuitPython

Install CircuitPython on W55RP20-EVB-Pico by referring to the link below.

- [**Installing CircuitPython**][link-installing_circuitPython]

Prototyping is easier than ever, with no software download required. Simply copy and edit the files to the **CIRCUITPY** drive, and repeat it.

![][link-circuitpy_1]

![][link-circuitpy_2]

Edit and save the code in **code.py**, your code will run on the W55RP20-EVB-Pico.

Let's check if CircuitPython is properly installed in your W55RP20-EVB-Pico with a simple blink example by referring to the link below.

- [**Blinky and a Button**][link-blinky_and_a_button]



<a name="setup_libraries"></a>
### Setup Libraries

To use Wi-Fi and additional functions of W55RP20-EVB-Pico, copy and paste the libraries included in the '[**WIZnet-ioNIC-Circuitpython/libraries**][link-libraries]' directory into the '**CIRCUITPY/lib**' directory.

![][link-copy_and_paste_library]



<a name="example_structure"></a>
## Example Structure

Examples are available at '[**W55RP20-EVB-Pico-CircuitPython/examples**][link-examples]' directory. As of now, following examples are provided.

- Loopback
- ping



<a name="example_testing"></a>
## Example Testing

Please refer to 'README.md' in each example directory to detail guide for testing examples.



<!--
Link
-->

[link-w55rp20-evb-pico]: https://docs.wiznet.io/Product/ioNIC/W55RP20/w55rp20-evb-pico
[link-installing_circuitpython]: https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython/circuitpython
[link-circuitpy_1]: https://github.com/WIZnet-ioNIC/WIZnet-ioNIC-Circuitpython/blob/main/static/images/getting_started/circuitpy_1.png
[link-circuitpy_2]: https://github.com/WIZnet-ioNIC/WIZnet-ioNIC-Circuitpython/blob/main/static/images/getting_started/circuitpy_2.png
[link-blinky_and_a_button]: https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython/blinky-and-a-button
[link-libraries]: https://github.com/WIZnet-ioNIC/WIZnet-ioNIC-Circuitpython/tree/main/libraries
[link-copy_and_paste_library]: 
[link-examples]: https://github.com/WIZnet-ioNIC/WIZnet-ioNIC-Circuitpython/tree/main/example
