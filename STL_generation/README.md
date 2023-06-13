# Script for generating STL files for new leg models for RealAnt robot platform
This script is designed to generate STL (Standard Tessellation Language) files for 3D printing. It takes input parameters such as leg length and radius of the leg tip to create the desired 3D model and export it as an STL file.

## Prerequisites
Before running the script, ensure that you have:
1. __FreeCAD 0.20.2__. It is an open-source parametric 3D modeler that is used for generating the 3D model.
You can install FreeCAD from the [official website](https://www.freecad.org/).
2. Original STL file **_leg_v8.stl_** from [realant repository](https://github.com/OteRobotics/realant/tree/master/stl)

## Usage
To use the script, follow these steps:
1. Download the script to your local machine
2. Open FreeCAD 0.20.2
3. Open the script in FreeCAD
	* File &rarr; Open
4. Edit the parameters in the beginning of the file to achieve desired dimensions and to specify file paths.
5. Once you have entered all the required parameters, press play from the top bar and the script will generate the STL file based on your input.

After generating the STL file, 

## Parameters

The script supports the following parameters:
* `length`: Length of the leg in millimeters.
* `tip_radius`: Radius of the rounded tip of the leg in millimeters.
* `orig_file`: Path to the original _leg_v8.stl_ file.
* `new_file`: Path where the resulting new STL file should be saved.

## Notes
* Make sure to provide the dimensions in millimeters (mm) as the script assumes millimeter units.
* The generated STL files can be opened and sliced in 3D printing software for further adjustments and printing.
* Make sure to rename or remove the produced STL file before running the script again. Otherwise it will not work correctly.
