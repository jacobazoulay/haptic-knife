# Haptic and Visual Rendering of Vegetable Cutting Knife

This project creates a haptic feedback display to simulate the experience of chopping a vegetable using a two degree of freedom system. The display has two motorized lever arms mounted on the back wall of a box: one horizontal arm controlled by the simulated vegetable that moves along the cutting board in order to position the vegetable under the knife, and one vertical arm with a 3D printed knife handle for cutting. The display was accompanied by a real time visual and auditory rendering.

This is the final project for ME327 Design and Control of Haptic Systems course at Stanford University.

The final paper can be found [here](http://charm.stanford.edu/ME327/Group1).

## Project Codebase

Two arduino micro-controllers with magnetoresistive (MR) sensor are used to control the two DC motors and serve as lever arm position sensors. Code has been written in the Arduino language and can be found in the arduino/ subdirectory of the repo.

All visual display is rendered using Processing. Serial data written by the Arduinos is read by Processing from the serial port. This position data is then used to render corresponding visualizations of the physical haptic device.

## Directory Structure

- **assets**: images for documentation
- **CAD-models**: all CAD models of the physical device.
- **arduino**:
	- [haptic-knife](https://github.com/jacobazoulay/haptic-knife/tree/main/arduino/haptic-knife "haptic-knife"): source code for knife haptic feedback rendering
	- [support](https://github.com/jacobazoulay/haptic-knife/tree/main/arduino/support "support"): miscellaneous motor control scripts and templates 
- **processing**: 
	- [haptic-knife-render](https://github.com/jacobazoulay/haptic-knife/tree/main/processing/haptic-knife-render "haptic-knife-render"): source code for visual knife rendering
	- support: miscellaneous templates

## Project Details
### Device Setup
![Annotated box](/assets/images/annotated_box.png)

### Visual Rendering
![Visual rendering](/assets/images/one_cut.png)

### Box Model
![Box model](/assets/images/box_assy.png)