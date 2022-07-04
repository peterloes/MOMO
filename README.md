# Multifunctional Mainboard to Observe and Manipulate Organisms (MOMO)
[\[paper\]](https://besjournals.onlinelibrary.wiley.com/doi/full/10.1111/1365-2656.13160) 2.3 Nest and feeder monitoring system

Authors: Loës P., Skripsky P., Kempenaers B. (2019) [![DOI](https://zenodo.org/badge/210341799.svg)](https://zenodo.org/badge/latestdoi/210341799)

Platform for Animal Observation and Manipulation 

An efficient system to monitor and control animal behaviour in wildlife
as well as in laboratory settings

- Targeted manipulation of 1000 and more individuals
- Supports a broad range of technical applications like e.g. traps, gates, feeders, lights, cameras
-	Compatible with different RFID systems and triggers
-	Low power consumption and specific data collection lead to long lasting collection periods
-	Applicable world-wide due to its compatibility with different types of radio clock receivers 
-	High observation quality through low disturbance to the animals
- Supports data-transfer to external devices, such as personal computers

![My image](https://github.com/peterloes/MOMO/blob/master/Getting_Started_Tutorial/2_Electronic_board.jpg)

- Time Synchronization with atomic clock once a day to ensure optimal data quality
- Application Current Control twice a day
- Forecast for Battery-Change implemented
- Energy Bypass so that Date and Time are maintained even when changing battery
- Hyperterminal Output to get real-time data in the field
- Low-Power Device:sleep: 220µA@12V with Light_Barrier awake: 12mA@12V


![My image](https://github.com/peterloes/MOMO/blob/master/Getting_Started_Tutorial/1_Feeder.jpg)

Bird Feeder Application:

https://github.com/peterloes/MOMO/blob/master/Getting_Started_Tutorial/1_poster_overview_3.pdf

Raw data on SD Card:

https://github.com/peterloes/MOMO/blob/master/Getting_Started_Tutorial/6_rawdata_BOX1005.TXT

Configuration data on SD Card:

https://github.com/peterloes/MOMO/blob/master/Software/CONFIG.TXT

Optional components: 

https://github.com/peterloes/Light_Barrier

https://github.com/peterloes/Servo_Engine

https://github.com/peterloes/Linear_Engine

https://github.com/peterloes/Booter_RFID-MS_MOMO_TAMDL

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL

Manufacture:

https://github.com/peterloes/MOMO/blob/master/Getting_Started_Tutorial/5_Supplier.txt

EXTENSIONS

Application Scales: Outdoor weighing system for animals include RFID-reader

https://github.com/peterloes/Scales

Application Audio: Record and playback soundfiles triggered by (RFID) pit-tag numbers

https://github.com/peterloes/Audio



C is the Greenest Programming Language

To achieve its power and energy-efficiency features, EFM32 products utilize ultralow active and idle power,
fast wakeup and processing times, and most important, the ability to intelligently interact with peripherals
and sensors autonomously without waking up the CPU and consuming more power.
