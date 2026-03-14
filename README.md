# Cooee (A LoRa Phone)                           
An open-source phone that can operate without GSM or LTE coverage. Utilising LoRa modulation and mesh networking to enable long range "off-grid" communication.

Come get involved or just chat here! -> [Discord](https://discord.gg/eWgrdjpTFj)
![Cooee_renders1](https://github.com/user-attachments/assets/be2b35a4-67d3-4329-a7a9-1cc6b175654d)


## Target Utilities/Functions
At time of writing (Jan 2026) Popular LoRa mesh networks only aim to support passing of text messages for public and private chat. Cooee aims to provide more of phone the features that consumers are used to

* __Voice calls:__ using 'Codec2' voice can be heavily compressed into small enough packets as to be supported by ~900MHz LoRa bandwidth limitations. Due to the delay that would happen if these packets needed to be retransmitted, calls would likely only be permitted between devices that have only 1 or 0 repeaters required between them.

* __Navigation:__ Using the data from OpenStreetMaps https://www.openstreetmap.org/ pre-loaded onto the device SD card a GPS reciever and infered location data based on signal strength to fixed repeaters of known location, the Cooee device should be able to locate itself well enough for navigation without ever needing a mobile data connection. 

* __Sending images and files:__
While LoRa bandwidth will likely never support sending large files (videos etc) piecemeal file transfers of compressed images (jpgs) should be possible

![Cooee_renders2](https://github.com/user-attachments/assets/19ba857a-2182-4fc4-b3ba-01dcd55f5894)

## Applications / Uses: 
The Cooee should ideally contribute to solving the following problems
  * Distress calls for help in remote locations
  * Basic communication in areas without GSM/LTE infrastructure
  * Dissemination of natural disaster and emergency warnings
  * Marking/Logging GPS locations. Users could tag and share hazards such as landmines (UXO/ERW), fires, vehicle collisions, chemical spills or dangerous wildlife.
  * Navigation in areas without GSM/LTE infrastructure

## Philosophy: 
My dream for this device is that is could demonstrate my ideals for what the future of harware aught to look like. This means deliberately achieving aims in ways which are not the 'best practice', such as;
 * Keeping code simple, using the Arduino framework isntead of ESP-IDF. The idea being that this increases how many people around the world would be able to understand and modify the device firmware, helping ensure that it is kept on track with providing maximum utility to the lives of those people who own one.
 * No electronic components smaller than 805 (no micro-soldering), this allows people to hand solder in repairs and hand assembly the PCB's with minimal equipement
 * SMT components on one side of the PCB only. Again allowing for easy assembly and repair without specialized gear
 * Plastic parts moulded from post consumer waste. Many plastics today claim to be 'recycled' when really they are just 'factory seconds', they have never used and discarded. I would like the plastic components of the Cooee to be injection moulded from recycld post consumer PET bottled pulled from waterways. Many countries in SEA are flooded with drink bottles made of PET. an easily recycled thermoplastic.
 * Hardware ownership. Increasingly, modern mobile devices are not really the property of the customer who paid hundreds, sometimes thousands of dollars to 'own' it. They are not free to modify it or install custom firmware, even creating custom firmware is a task far beyond the capabilities of most. This leaves people at the mercy of whatever bloatware the manufacturers have bundled. This bundled software often; listens to you at all times, sends transcripts of your conversations to corporate servers, advertises products at you, insists on volunteering additional personal details to enable use ect. I think it's high time that the cleverst people in the world put thier efforts towards creating machines that provide all of the utility that benefits us without any of the data harvesting and advertising. Imagine how nice it would be to use a navigator which simply puts your GPS position on a locally loaded map, it doesn't start suffocating your device by chewing up huge amounts of bandwidth or report everywhere you go to a server or try to encourage spending on nearby businesses. Imagine a navigator which just helps you nagivate in the most lightweight and unobtrusive way possible.
 This is the philisophy I want to see applied to all the functions of the Cooee software. 
