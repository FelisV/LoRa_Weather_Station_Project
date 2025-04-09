<p align="center">
    <img src="images/Weather_Station.png" alt="Cover Image" width="350" height="300">
</p>
  
<h1 align="center">LoRa Weather Station Project</h1>
<h2>Table of Contents</h2>
<ol>
    <li><a href="#introduction">Introduction</a></li>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#software">Software</a></li>
    <li><a href="#libraries-installation">Libraries installation</a></li>
    <li><a href="#sensor-module">Sensor Module</a></li>
    <li><a href="#Lora Interface">Lora Interface</a></li>
    <li><a href="#requirements">Requirements</a></li>
</ol>
  
<h2 id="introduction">Introduction</h2>
<p>
    The goal of this project is to develop a weather station based on Seeeduino LoRaWAN W/GPS, capable of collecting environmental data such as temperature, humidity, atmospheric pressure, and other information. 
    It uses LoRa technology, which has the significant advantage of wireless connectivity over long distances with low power consumption. 
    This allows for long-distance transmission of data via LoRa to the TTN server, where the data will be processed and visually displayed remotely.
</p>
  
<h2 id="hardware">Hardware</h2>
<p> 
    <img src="images/Hardware.png" alt="Hardware Image" width="300" height="200">
    <figcaption>Hardware used in the project, For more information, click the link
    <a href="https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/" target="_blank">Wiki Seeedstudio.</a>
      </figcaption>
</p>
  
<h2 id="software">Software</h2>
<ul>
    <li> Arduino IDE;</li>
    <li> Arduino libraries for sensor integration;</li>
    <li>Sensor Module (Contains the script and tests for collecting environmental data from the sensors);</li>
    <li>LoRa Interface (Contains the script responsible for LoRaWAN communication and data transmission);</li>
</ul>
  
<h2 id="libraries-installation">Libraries and Installation</h2>
<p>
    The libs/ folder stores all libraries used in the project, divided into:
</p>    
<ul>
    <li>external/ -> Contains external libraries;</li>
    <li>custom/ -> Includes custom libraries developed for this project to improve sensor integration;</li>
</ul>
<p>Installing Libraries in Arduino IDE:</p>
<ol>
    <li> Open Arduino IDE;</li>
    <li> Navigate to Sketch -> Include Library -> Add.ZIP Library...;</li>
    <li> Select the .zip file from libs/external and libs/custom;</li>
    <li> The libraries will now be available;</li>
</ol>    

<h2 id ="sensor-module">Sensor Module</h2>
<p>
    <p>The sensor module was developed to collect and analyze environmental data using the BMP280, DHT20 and GPS sensors. 
    Is structured into different folders for better organization and ease of use.  </p>
        Module Struture is organized as follows:
        
        |── tests/                       # Main folder for the tests
        |    ├── sensors/                # Main folder for the sensor project
        |    │   ├── main/               # Main folder containing the main test script
        |    │   ├── screenshots/        # Folder for all screenshots
        |    │   │   ├── all_sensors/    # Screenshots for all sensors combined
        |    │   │   ├── BMP280/         # Screenshots for BMP280 tests
        |    │   │   ├── DHT20/          # Screenshots for DHT20 tests
        |    │   │   ├── tests/          # Main folder for the tests
    
        About the other folders:
            main:
                The main/ folder contains the main test script inside. This script allows:
                    - Testing each sensor individually;
                    - Combine and test the sensors simultaneously, as well as the sensor readings, to verify proper integration;
            screenshots:
                The screenshots/ folder contains screenshots of tests performed, organized into subfolders:
                    - all_sensors/ -> Contains results of tests where all sensors are working together;
                    - BMP280/ -> Screenshots of BMP280 tests (modes, standby, oversampling, pressure and temperature readings);
                    - DHT20/ ->  Screenshots of DHT20 tests (temperature and humidity readings);
</p>
  
<h2 id="Lora Interface">Lora Interface</h2>
  <p>

  </p>
<h2 id="Requirements">Requirements</h2>
<ul>
    <li> Arduino IDE to compile and upload the code;</li>
    <li> Necessary libraries (found in the libs/ folder) to install for their use;</li>
</ul>                
  
  
