# STM32_BLE_Custom_Server_UART
CubeMx implementation of Bluetooth LE driving. Project showcases reception of strings using an STM32WB5MM Discovery board and a smartphone with ST BLE Toolbox installed. This is the first within a series of projects around BLE.

## General description
Another project, another mcu…

Yes, we will be looking at our first dual core mcu, more specifically, the WB5MM and its associated discovery board.

Why the change AGAIN? Well, because we are talking about a c.a. 30 EUR board (in 2025) but one that is readily equipped with a bunch of goodies, including an in-built OLED screen, a microphone and…a ++complete Bluetooth module!

What we will be doing here in this section is sending a string from the phone and then print it out on the OLED screen. Simple but useful to showcase Bluetooth Low Energy (BLE) communication using mcu-s. Later on, we will be building up specific servers and clients to match our needs, albeit those will be done in a different project. Here we will only cover the basic theory of BLE and make a simple UART pipe.

### BLE
What is so special about BLE?
Let’s just forget all the different configurations like BLE, Bluetooth Classic/Standard or Zigbee for a second. In practical sense, Bluetooth hardware is a radio module that can flexibly change its communication frequency through modulation around the 2.4 GHz mark (more precisely, between 2.4 GHz and 2.48 GHz). This means that the same radio module can communicate on multiple different frequencies (called channels with altogether 39 of them in the range indicated above) and even actively change/swap during the communication between these channels. As a matter of fact, we only use one channels for 625 microseconds and then go to another one.

The sequence of channels the central device (or client) uses/hops around is communicated to the peripheral (or server) upon contact. This means that Bluetooth is difficult to hack into or spoof. Unfortunately, it has significantly less bandwidth than. Say, WiFi – with 6.0 BLE, around 0.25 MB/s – so the type of data transfers we can do with BLE is practically limited.

Anyway, if it wasn’t obvious, we are using some rather complicated hardware here that must be driven by an associated communication stack (or “app” as ST calls it), similar to how USB has its own stack. This stack though isn’t just simply driving the hardware, it also defines the protocol, which can be various types, like Bluetooth Standard, Low Energy or Zigbee.

Anyway-anyway, we will be using two cores to run BLE (so, Bluetooth Low Energy) in a STM32WB5 mcu, where the coprocessor (the small M0 bundled with the M4) will be dedicated to driving the hardware and the main cpu (the M4) will be generating the packets/messages (i.e. feed the M0 so it will churn out the right type of signal). Be aware that while the FUS of the coprocessor (M0’s bootloader pretty much) is the same across the board for all protocols, the driving stack will depend on the protocol we are choosing.

#### IPCC, HSEM, RF and a few words about the WB line
Writing our own Bluetooth stack (on the coprocessor) and packet generator (the “app”) is a no-go, obviously, so we will be using the versions provided by ST, more precisely, the ST BLE “app”, conveniently available as a middleware called WPAN in CubeMx. Of note, CubeMx will not allow us to include the WPAN middleware unless we have some peripheral enabled already (namely, the associated RCC clock running at 32 MHz, RTC, plus three more elements called IPCC, HSEM and RF).

Firstly, just a few lines of clarification on the WB line of mcus: yes, they are M0-M4 dual core devices, but they actually share a whole lot of things, such as the physical FLASH and SRAM memory (though access is restricted to one cpu or the other), clocking and multiple additional peripherals (RTC and AES comes to mind). We are having these the two cores running in parallel and will have to synchronize them as well as facilitate communication between them without corrupting data.

(I suggest giving a quick look at my RTOS project since many concepts mentioned there have direct parallels to what we have here: while there using RTOS we were “mimicking” parallel processes on a single core, here we will have actual hardware-based parallel processes on two separate cores.)

The two most important elements are:

- IPCC, which is what we used to call “queues” in RTOS. They are data channels that are small FIFOs – here, one-element hardware-controlled FIFOs – facilitating data transfer between the two cores. To recap, we need these since otherwise it may happen that we are reading out a value from memory with one core while the other one is changing the exact same value EXACTLY the same time, thus corrupting our data. IPCC is purely hardware-based with internal flags and interrupts to indicate when one of the 12 IPCC channels are loaded or free. The channels can be simplex (either Tx or Tx) or half-duplex (a Tx followed by an immediate Rx). Each channel is managed separate by the flags and the associated interrupts plus their handlers. The interrupts are only used in case the channels are found to be locked – then the interrupt mask is removed - otherwise an unused channel is masked. The sequence of locking/unlocking writing/reading depends on if we are doing simplex or half-duplex communication. Mind, the memory the IPCC is actually using as FIFO is NOT part of the IPCC and thus can be prone to corruption.
- HSEM is what semaphores were in RTOS but applied to peripherals instead. It allows a core to call a shared peripheral and then not allow access to it by the other core until the first core releases the associated semaphore. 

To be clear, I can only give such a surface level explanation of what these elements are doing since both the IPCC and HSEM are buried deep in the BLE stack. I assume, the BLE stack juggles the flags and the interrupts as it loads the IPCC and runs the HSEM.
Apart from the IPCC and the HSEM, a third Bluetooth-specific hardware peripheral we will be activating in CubeMX to allow the WPAN middleware is RF.  RF is the radio module. We literally have nothing to set up on it, albeit it must be activated in the CubeMx. This means that the clocking for the RF module must be activated and set by either running the LSE or a dialled down HSE (this is why the HSE must be 32 MHz, so we will be able to DIV10 it to the same rough frequency as an LSE). Both will be figured out by the CubeMX (or just check the RCC_CSR and RCC_EXTCFGR registers, plus the enable on the APB3). Funnily, the RFinit function CubeMx generates using HAL doesn’t do anything, we only need to set the RF clocking properly to make RF work by hand.
Lastly, the RTC also needs to be enabled to allow the WPAN middleware to work. This will be there to provide the “TimerServer”, the timing for the WPAN library. This is again an integral part of the stack like the IPCC and HSEM and removing it breaks the WPAN. The RTS can be clocked through LSE or LSI, as long as it is 32kHz.

#### BLE hardware alternatives
As it might already be clear, I am a big fan of ST chips, mostly because – unlike other options on the market – they are very easy to set up and do not need any extensive infrastructure to program and debug thanks to their in-built debuggers. Nevertheless, they aren’t easy to get into if you aren’t about all that low-level bit flipping.

In general, if someone is interested in setting up their own BLE, I generally would not recommend using the ST solution for the simple reason that there is very little information on it available, little documentation and limited support. Alternatives can be way easier to implement, the libraries doing all the heavy lifting for you in the background.

Anyway, within the domain of microcontrollers and programming in C, there are practically two alternatives worth mentioning, both having strong support in the Arduino IDE. (I skip the Raspberry Pico since it is mostly python-based.) These are:

- ESP32 is pretty much the go-to solution these days for hobbyists for the simple reason that the chips are very cheap and come with both BLE and WiFi included. Support in the Arduino (well, Adafruit…) community is also very high, so it is very likely one can find an existing code already on github without needing to go to deep into coding. The problem with ESP32 though is the chip’s size and power consumption which both are on the greater end, not making them good for battery-powered small footprint applications.
- Nordic chips, more precisely the NRF51 and NRF52 lines. They are a lot smaller and more efficient than ESP32, making them ideal for battery applications. NRF52 chips are the ones used in the official Arduino Nano BLE so support is extensive. Adafruit also has their own NRF52 boards, so there is an alternative to the Arduino version in case we aren’t happy with it. Adafruit also sells multiple boards with the NR51 installed which is a smaller, uni-core version of the dual-core NRF52. Since NRH51 are uni-core (so, only the co-processor is included in the package), an external core will be necessary to interface with them and generate the protocol (this does not apply to the NRF52 which can run both the co-processor and the app code). Be aware that – according to Adafruit – they don’t have a reliable “client” code for the NRF51, so they cannot be used as central devices using existing libraries. NRF51 chips will – by nature – be less efficient than dual cores due to the inter-core communication relying on conventional protocols such as UART or SPI instead of something dedicated like the IPCC/HSEM in the WB5.
  
Of note, despite doing the exact same thing, we are using separate stack libraries depending on the device: the “NimBLE” library for the ESP32, the “ArduinoBLE” for the Nano and the “bluefruit” for the Adafruit. These stacks are not interoperable amongst these devices.

#### The package, the GAP and the GATT
Now that we have the hardware theory covered, we need to talk about, what we will be feeding into it.

From a purely physical point of view, the data package within BLE consists of a 72-bit access code, a 54-bit header and a payload. The access code does the recognition and synchronisation between devices, the header describes the package type we are sending, and the payload stores the data itself. The payload size can vary depending on the package type – say, a simple ACK from a device back to a client will be just a few bits, while sending audio from the client to the server might fill up the entire payload (at maximum, 8kbits). At any rate, we don’t need to bother ourselves with the packages: they will be generated automatically thanks to the WPAN library, which is ST's BLE stack (see "bluefruit" for the Adafruit version). Also, we can have multiple packets sent over during the same transmission. Anyway, everything is defined prior to the communication starting.

But, what will the WPAN generate for us? It will generate GAP and GATT.

GAP (Generic Profile Access) is, technically speaking, what defines if a BLE-enabled device will be a server or a client and, if it is a server, what kind of data specs/device name will it be advertising. The advertising name will be the name of the device when scanning, meaning that it will be used to recognise and connect to it. Mind, we can add additional information to the advertising data such as device information, UUIDs (or Universal Unique Identifier, see the follow-up project for those), advertising intervals and so on and all of that will be sent around with the name we have given to our device.

GATT (Generic Attribute) are a set of transactions that will build the communication protocol between our two devices. The server will be the peripheral, holding the all the GATT characteristic of the device and the client will be the central device (phone) which will regularly request information or activity from the peripheral. The client is expected to honour the timing (connection interval) and the communication demands of the server, albeit it can ignore them should the channel be deemed noisy or unavailable.  GATT has three different elements:

- A GATT profile is a high-level collection of services between the two devices. Profiles are pretty much applications and one can found a whole bunch of them as official profiles on the Bluetooth website. They are built up from services. We won't be really touching open these.
- GATT services are the “functions” within GATT. Each can have one or more characteristics. Services have their own ID which then helps two devices to recognise and synchronise their activities.
- GATT characteristics are the lowest level elements in GATT. They can be paralleled with variables (i.e., collecting of data points). They also have their own ID.

An example for a GATT service is a custom “UART-type coms”, which would consist of a “TX” and an “RX” characteristic where TX would have write privileges and RX as read only. We will do only the Tx part here so as to stay with only one characteristic.

## Previous relevant projects
I am going through the related Phil’s Lab project and elaborate a bit on the documentation he is referring to: 

[STM32 Bluetooth Firmware Tutorial (Bring-Up) - Phil's Lab #129](https://www.youtube.com/watch?v=-xYoI84zJew)

## To read
I also suggest checking out the official tutorials, though Phil is definitely doing a better job at explaining them compared to the peeps at ST:

[STM32WB OLT - 1. Introduction Welcome session](https://www.youtube.com/watch?v=OIVx817WGf0&list=PLnMKNibPkDnGkMxFkRArr9uOq_Es_a7vG&index=3)

Lastly, I have found this video particularly useful understanding, how Bluetooth works in general:

[How does Bluetooth Work?](https://www.youtube.com/watch?v=1I1vxu5qIUM)

## Particularities
### Setting up the WB5's co-processor
After playing around with RTOS on other devices, what struct me as strange was that even though we have two cores, we are not given direct access to both of them by the STM32 IDE. On the contrary, we will only be able to interface with M4 cpu unless we activate a set of debugging options. Similarly, when we are uploading anything to the WB5 through the IDE, we will only upload to the FLASH of the M4 and NOT to the co-processor M0. I suppose they wanted to remain sure that people will only use that second core for what it is intended and not do some parallel processing with it...

Whatever was their reasons to insulated the M0 for access, we do need to load the it with the necessary stack information, otherwise we won't be able to physically generate our BLE signal through the antenna.

The workaround to the issue is to write DIRECTLY to a memory section within the WB5 device using the STM32 Programmer. Now, the STM32 Programmer is just a programming tool that allows us to load existing (some form of hex-formatted) code onto a device without the need to compile the code again from the C source text. This makes it a lot faster for industrial purposes...not to mention, it seems to be significantly more reliable as a programming tool for custom boards.

Anyway, what we can do with the Programmer is connect to our WB5 board through the attached STLink and then load any kind of code to any part of the FLASH memory (mind, the FLASH is shared between the two cores) within the device. Question is, what code and where? 

For a step-by-step installation guide, check the Phil's Lab Youtube video shared above.

#### Don't FUS about it
The FUS - Firmware Upgrade Stack - is one of the binaries we will need to load into our co-processor using the ST Programmer. As far as I can tell, the FUS is a type of bootloader for the M0 that handles the BLE stack loading as well as some form of encryption. FUS the same for all possible stacks and WB devices, so just pick the newest one available for download from github (https://github.com/STMicroelectronics/STM32CubeWB/tree/master/Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x).

To know where to put the FUS, we need to check the "Release notes" html file also available here in github. In the "Firmware Upgrade Services Binary Table:" section we can see that the FUS is supposed to go to address "0x080EE00" in the WB5xxG device (which is what is on our disco), so that' where we need to load it with the ST Programmer.

#### Stack the BLE
The second binary we will need is the communication stack. Following the Phil's Lab example, I downloaded the latest BLE full stack and then uploaded it to address 0x080D0000 (different for the WB5 than Phil’s solution, taken from the Release notes html file).

Once done, it is recommended to run the FUS in the programmer and check if everything is properly updated. If so, we are done with the co-processor and can move to the main processor by using the IDE, as usual.

Of note, the entire messing around with the co-processor is absent when using Arduino IDE. For Nordic and ESP32 chips, this part has been conveniently implemented directly into the BLE library.

### Setting up a custom server
Let’s do a custom GAP/GATT solution where we send over a simple string (“hello world”) to the WB5 from a phone. Mind, we will be generating a peripheral device (a server) here that will behave as a simple UART pipe, sending over a string.

We need to activate WPAN library stack for the M4 by enabling all the requested peripherals (IPCC, HSEM, RF, RTC).

Within CubeMX, in order to bring up GAP and GATT, the device we are setting up must be defined as server by deactivating the present P2P server definition, pick the BLE application types as “server” and bring in a custom template. We won’t be using the existing server templates, for now.

To set the GAP up, we merely need to set the LOCAL_NAME element to YES and give a name to the device. This name will be what our device will be seen as when we are doing a BLE scan. We won’t add any additional information to the advertising.

For GATT, we will need to define the name (and the number) of our custom service(s) in “BLE_GATT”. Once done, we will have a new tab appearing that will represent this service and allow us to set its characteristics/data containers and the number of said characteristics. We can then tell the name of the characteristic, the size of it (how many elements will it be dealing with), add a UUID to our custom service as well as the characteristic, what properties it will have (write to or read from the server, or maybe a simple notification), what permissions it will have and when will the characteristic generate an event (the GATT version of an interrupt). The “Custom_STM_Event_Handler” function within the “custom_stm.c” source code will then be called whenever the event is generated, i.e. when the characteristics “fires”.

Please note that the names of the services and characteristics are purely indicative for CubeMx use and will not emerge as distinct values within the code (they will be within the advertising information though). WPAN will instead just call them by number, such as Service 1 and Service 1 Characteristic 1. This is a bit confusing since other libraries (such as the “bluefruit.h”) runs on the names of these services and characteristics.

Mind, since we are making a custom service with custom characteristics, we will not use any UUIDs and just leave it blank. We will work in this more in the second project.

Here, we will be sending over 120 bytes all the time (60 ASCII characters), just for convenience’s sake: 60 characters will fit comfortably on the WB5’s LCD screen within 16 point letter size. We will need to set this value within our custom characteristic's properties. Setting "variable" length will allow for the transfer to occur for any strings that are shorter than that as well. Please be aware that the "Event" mentioned above will only fire if the transfer complies and makes our characteristic is "filled". Sending more data packets than the maximum limit and overflowing the characteristic leads to the stack crashing for me.

### Setting up a client
Our BLE client for this project will be just a smartphone, running the ST app called "ST BLE toolbox". I won't get into this, the app is pretty straight forwards and provides us a simple serial interface to play with.

Mind, there is another ST app called the "ST BLE Sensor" which provides some mor complicated interfacing options. It is also the one we need if we want to run some of the ST-provides example codes on the WB5.

## User guide
This custom code is rather simple: whenever we are sending a string to the WB5MM, it will go and write it on the screen. The string length limit is 12 characters.

Of note, I am merely sharing the source code I have directly modified after CubeMx has generated the files. If one follows Phil’s lead and modify everything as I have described here, the output should work.

IMPORTANT!!! The "custom_stm.c" source code DOES NOT go into the the src folder of the code! It goes to "STM32_WPAN/App"! with all the other BLE-related files.

I am also sharing the source code to run the OLED screen. Unfortunately, unlike previous disco screens - like the one on the F429ZI or the F412G - the screen on the WB5 is an OLED . This makes it not compatible with previous TFT screen drive philosophies (no convenient IO layer found to be loaded). It is also soldered/attached directly to the board so no alternative boards can be used to test the screen and then port a solution from there. In the end I took one from github (user "libdriver" https://github.com/libdriver/ssd1315) and tailored it to run on the WB5. Of note, the "libdriver" solution has practically no explanation given to it, nor does it indicate clearly where the different layers (main/IO/driver/peripheral, see FSMC or LTDC screen projects) should attach. I have added some explanation here and there, that should help.

## Conclusion
We have successfully set up a custom server that will react to an incoming data string by printing it our on its OLED screen.

This is just the first part of a string of projects to figure out BLE. Here we have set up the hardware and got familiar with the most basic concepts. The next parts will get steeper…

