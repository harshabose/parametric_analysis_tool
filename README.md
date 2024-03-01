# parametric_analysis_tool
A simple and effective aircraft parametric analysis tool.

This is a small repository of a larger project in works which aims to automate pre-conceptual and conceptual design process of a fixed-wing aircraft. The aim of the project is to allow the user to generate rapid analysis data for trade-off studies allowing for faster and leaner conceptual design phases. The scope of this project is primarily focused of small scale projects which may include modern Unmanned Aerial Vehicles (UAV) and mid-sized eVTOL aircraft (conventional aircraft are not supported, though support for inclusion of additional design criteria are given in the header file).

The current repository houses the parametric analysis toolkit which allows for a rapid and accurate parametric analysis of an aircraft concpet to obtain optimal values for Thrust-to-Weight ratio and Weight-to-Surface Area ratio. This step is often the foundational step of any aircraft design stage and automating it gives the desingner a sleek advantage.

## How to Use?
It is a simple header-only file, located in [include/parametricAnalysis.h](https://github.com/harshabose/parametric_analysis_tool/blob/main/include/parametricAnalysis.h). To integrate this tool into your project, simply copy the header file into your working directory and include it in your source code. The process is designed to be straightforward, removing unnecessary complexities.

For demonstration purposes, additional files are provided within this repository to showcase a test case. Follow the instructions below to execute a sample analysis, which will utilise design, atmospheric, and operational conditions specified in [src/parametric_analysis.cpp](https://github.com/harshabose/parametric_analysis_tool/blob/main/src/parametric_analysis.cpp). Upon completion, the program will display the optimal T/W and W/S ratios.

``` bash
  git clone https://github.com/harshabose/parametric_analysis_tool.git
  mkdir build && cd build
  cmake ..
  make
  ./parametric_analysis_tool_kit
```

The execution of these steps will initiate a test run, culminating in the presentation of optimal values for key design ratios, thus illustrating the practical utility of the parametric analysis toolkit in the conceptual aircraft design cycle.
