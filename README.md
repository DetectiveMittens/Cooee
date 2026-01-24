# Cooee (A LoRa Phone)
An open-source phone that can operate without GSM or LTE coverage. Utilising LoRa modulation and mesh networking to enable long range "off-grid" communication

## Target Utilities/Functions
At time of writing (Jan 2026) Popular LoRa mesh networks only aim to support passing of text messages for public and private chat. Cooee aims to provide more of phone the features that consumers are used to

* __Voice calls:__ using 'Codec2' voice can be heavily compressed into small enough packets as to be supported by ~900MHz LoRa bandwidth limitations. Due to the delay that would happen if these packets needed to be retransmitted, calls would likely only be permitted between devices that have only 1 or 0 repeaters required between them.

* __Navigation:__ Using the data from OpenStreetMaps https://www.openstreetmap.org/ pre-loaded onto the device SD card a GPS reciever and infered location data based on signal strength to fixed repeaters of known location, the Cooee device should be able to locate itself well enough for navigation without ever needing a mobile data connection. 

* __Sending images and files:__
While LoRa bandwidth will likely never support sending large files (videos etc) piecemeal file transfers of compressed images (jpgs) should be possible

## Applications / Uses: 
The Cooee should ideally contribute to solving the following problems
  * Calling for distress in remote locations
  * Basic communication in areas without GSM/LTE infrastructure
  * Dissemination of natural disaster and emergency warnings
  * Marking/Logging GPS locations. Users could tag and share hazards such as landmines (UXO/ERW), fires, vehicle collisions, chemical spills or dangerous wildlife.
  * Navigation in areas without GSM/LTE infrastructure

