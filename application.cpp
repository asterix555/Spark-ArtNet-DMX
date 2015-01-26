#include "application.h"
/*
 Used Spark Core Pins

 TX (PA2)   DI      (USART2)
 A2 (PA4)   DE/!RE  (Not really necessary if you only want to continuously send DMX)
 D7 (PA13)  Blue LED
*/

SYSTEM_MODE(SEMI_AUTOMATIC);
// SYSTEM_MODE(AUTOMATIC);

uint8_t dmxData[512]; // Index 0 is DMX channel 1
uint8_t dmxFade[512]; // DMX-Fade-To Values
uint8_t dmxFadingTime[512]; // DMX-Fade-To Values

unsigned long timeNow, timeLast;


#define ENABLE_DMX 1
#define ENABLE_TCP 1
#define ENABLE_UDP 1
#define DEBUG_NET  0

#define DEBUG_DMX_COLS  8
#define DEBUG_DMX_ROWS  32

#define PACKETSIZE_MIN  2
#define PACKETSIZE_MAX  20
#define CHANNEL_SET 0
#define CHANNEL_FADE 1
#define TYPE_UDP 1
#define TYPE_TCP 2


#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x) (LOW_BYTE & x)
#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) )


#define toggle_runled() GPIOA->ODR ^= (1 << 13) // Toggle Sparks blue LED

#if ENABLE_DMX
#define toggle_usart2_de_re() GPIOA->ODR ^= 0x0010 // Toggle DE/!RE-lines
#define set_pa2_uart() GPIOA->CRL = 0x44334B44 // Set TX to USART
#define set_pa2_gpio() GPIOA->CRL = 0x44334344 // Set TX to GPIO

#define tim2_wait_usec(X) \
        TIM2->PSC = 71; \
        TIM2->ARR = X; \
        TIM2->CNT = 0; \
        TIM2->CR1 |= TIM_CR1_CEN; \
        while (TIM2->CNT != TIM2->ARR) { } \
        TIM2->CR1 &= ~TIM_CR1_CEN

#define usart2_tx_and_wait(X) \	
        USART2->DR = X; \
        while(!(USART2->SR & USART_FLAG_TC)) { }

// DMX TX Interrupt
#ifdef __cplusplus
extern "C" {
#endif
    extern void (*Wiring_TIM3_Interrupt_Handler)(void);
    extern void Wiring_TIM3_Interrupt_Handler_override(void);
    
#ifdef __cplusplus
}
#endif
#endif

unsigned int localPort = 3200; // artnet UDP port is by default 6454

#if ENABLE_TCP
TCPServer server = TCPServer(localPort);
TCPClient client;
#endif

#if ENABLE_UDP
UDP Udp;
#endif

void udpSend(String message) {
    Serial.print("> " + message);
    message += "\n";

#if DEBUG_NET
#if ENABLE_TCP
    if (client.connected()) {
        server.write(message.c_str());
    } else {
        Serial.print(" (not connected)");
    }
#endif

#if ENABLE_UDP
    IPAddress ipAddress = Udp.remoteIP();
    if (ipAddress) {
        int port = Udp.remotePort();
        Udp.beginPacket(ipAddress, port);
        Udp.write(message.c_str());
        Udp.endPacket();
    }
#endif

#endif
    Serial.println(" - Done");
}

#if ENABLE_DMX
void setupDMX()
{
    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // Enable USART2 clock
    RCC_APB2PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);
    
    // Enable TIM2 clock (for waiting in dmx_send)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);
    
    // Enable TIM3 clock (for dmx send interrupt)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);
    
    
    // Enable USART2
    RCC->APB1ENR |= RCC_APB1Periph_USART2;

    
    // Setup pins
    GPIO_InitTypeDef pinCfg;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    
    // USART2 TX Pin: Alternate function push pull
    pinCfg.GPIO_Mode = GPIO_Mode_AF_PP;
    pinCfg.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &pinCfg);
    
    // Configure LED pin
    pinCfg.GPIO_Mode = GPIO_Mode_Out_PP;
    pinCfg.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOA, &pinCfg);
    
    // Configure PA4 pin for output (DE/!RE)
    pinCfg.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &pinCfg);
    
    // Setup USART 2 for DMX TX
    USART_InitTypeDef usartCfg;
    usartCfg.USART_BaudRate = 250000;
    usartCfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartCfg.USART_Mode = USART_Mode_Tx;
    usartCfg.USART_Parity = USART_Parity_No;
    usartCfg.USART_StopBits = USART_StopBits_2;
    usartCfg.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &usartCfg);
    
    // enable USART2
    USART_Cmd(USART2, ENABLE);
    
    
    // Configure TIM3 for 25Hz interrupt
    Wiring_TIM3_Interrupt_Handler = Wiring_TIM3_Interrupt_Handler_override;
    
    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = 71;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 40000;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &timerInitStructure);
    
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    
    //Enable Timer Interrupt
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 2;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}
    #endif

void setup() {
        toggle_runled();
        // USB Serial for debugging
        Serial.begin(9600);

    #if ENABLE_DMX
        // DMX
        setupDMX();
    #endif
        // Control the RGB LED
        RGB.control(true);

        uint8_t pinNumber = 0;
        for (pinNumber = 0; pinNumber < 8; pinNumber++) {
            pinMode(D0 + pinNumber, OUTPUT);
        }
        pinMode(A0, OUTPUT);
        pinMode(A1, OUTPUT);
        pinMode(A2, OUTPUT);
        pinMode(A3, OUTPUT);
        /*
        pinMode(A4, OUTPUT);
        pinMode(A5, OUTPUT);
        pinMode(A6, OUTPUT);
        pinMode(A7, OUTPUT);
         */
        WiFi.on();
        WiFi.connect();

    #if ENABLE_UDP
    #endif
    }
    uint16_t n;
    int8_t udpStarted;

    void submitDmxData(uint16_t &dmxChannel, uint8_t &dmxValue, uint8_t &type, uint8_t & fadingTime) {
        if (dmxChannel && dmxChannel <= 512) {
            dmxChannel--;
            String message;
            if (type == CHANNEL_SET) {
                dmxData[dmxChannel] = dmxValue;
                message = "Setze";
            } else {
                message = "Blende";
            }
            dmxFade[dmxChannel] = dmxValue;

            if (fadingTime > 0) {
                dmxFadingTime[dmxChannel] = fadingTime;
            }
            udpSend(message + " Kanal " + String(dmxChannel + 1, DEC) + ", Wert: " + String(dmxValue, DEC));
        } else {
            udpSend("Kanal ungueltig: Kanal " + String(dmxChannel, DEC));
        }
    }

    void processPacket(int16_t &packetLen, uint8_t packetType) {
        uint8_t dmxValue = 0, fadingTime = 0, commandLen = 0, status = 0, type = 0;
        uint16_t dmxChannel = 0;
        char c = 0;

        // while (packetSize > PACKETSIZE_MIN && packetSize <= PACKETSIZE_MAX) // Check size to avoid unnecessary checks
        for (uint16_t pos = 0; pos < packetLen; pos++) {
            toggle_runled();


    #if ENABLE_TCP
            if (packetType == TYPE_TCP) {
                c = client.read();
            }
    #endif

    #if ENABLE_UDP
            if (packetType == TYPE_UDP) {
                c = Udp.read();
            }
    #endif

            commandLen++;

            if (c == ',') {
                // go to next status;
                // Serial.print(", ");
                status++;
            } else {
                if (c == ';'
                        || c == 0x04
                        || c == 0x0A
                        ) {
                    // Serial.println("  -- Seperator");
                    if (commandLen >= PACKETSIZE_MIN) {
                        status = 255;
                        udpSend("Finished command: Length = " + String(commandLen, DEC) + ")");
                    } else {
                        status = 0;
                    }
                } else {
                    // Serial.print(c);
                }
                if (commandLen > 8) {
                    status = 0;
                    udpSend("Invalid Packet: Length = " + String(commandLen, DEC) + ")");
                }

                switch (status) {
                    case 0:
                        // Set Mode
                        dmxChannel = 0;
                        dmxValue = 0;
                        fadingTime = 0;
                        commandLen = 0;
                        status++;

                        switch (c) {
                            case 'S':
                                type = CHANNEL_SET;
                                break;

                            case 'F':
                                type = CHANNEL_FADE;
                                break;

                            default:
                                udpSend("Unknown Packet: " + String(c) + " (HEX: 0x" + String(c, HEX) + " DEC: " + String(c, DEC) + ")");
                                status = 0;
                                break;

                        }
                        break;

                    case 1:
                        // Set Channel
                        if (c >= '0' && c <= '9') // Detect valid Number
                        {
                            dmxChannel = dmxChannel * 10 + c - '0';
                        } else {
                            udpSend("Invalid Channel");
                        }
                        break;

                    case 2:
                        // Set Value
                        if (c >= '0' && c <= '9') // Detect valid Number
                        {
                            dmxValue = dmxValue * 10 + c - '0';
                        }
                        break;

                    case 3:
                        // Set Fading time
                        if (c >= '0' && c <= '9') // Detect valid Number
                        {
                            fadingTime = fadingTime * 10 + c - '0';
                        }
                        break;
                }

                if (status == 255 || pos == packetLen - 1) {
                    // Submit DMX Data
                    submitDmxData(dmxChannel, dmxValue, type, fadingTime);
                    status = 0;
                }
            }
            toggle_runled();
        }
    }

    void checkDmxServer(uint8_t packetType) {

        int16_t packetLen = 0;

    #if ENABLE_TCP
        if (packetType == TYPE_TCP) {
            if (client.connected()) {
                packetLen = client.available();
            } else {
                client = server.available();
            }
        }
    #endif

    #if ENABLE_UDP
        if (packetType == TYPE_UDP) {
            packetLen = Udp.parsePacket();
        }
    #endif

        if (packetLen) {
            Serial.print("Client Connected: ");

    #if ENABLE_UDP
            if (packetType == TYPE_UDP) {
                Serial.print("UDP, Remote IP: ");
                Serial.print(Udp.remoteIP());
            }
    #endif
    #if ENABLE_TCP
            if (packetType == TYPE_TCP) {
                Serial.print("TCP");
            }
    #endif

            Serial.print(", Length: ");
            Serial.print(packetLen, DEC);
            Serial.println(" bytes");

            if (packetLen >= PACKETSIZE_MIN && packetLen <= PACKETSIZE_MAX) {

                // udpSend("Evaluating Packet (Size: " + String(packetLen, DEC) + ")");
                processPacket(packetLen, packetType);
            }

    #if ENABLE_TCP
            if (packetType == TYPE_UDP) {
                client.flush();
                client.stop();
            }
    #endif

    #if ENABLE_UDP
            if (packetType == TYPE_UDP) {
                Udp.flush();
            }
    #endif
        }
    }

    void printDmxValue(uint16_t value) {
        if (value < 10) Serial.print(" ");
        if (value < 100) Serial.print(" ");
        Serial.print(value, DEC);
    }

    void printStatus() {
        Serial.println();
        Serial.println("================================================================");
        Serial.println("|                NETWORK STATUS ---- SPARK CORE                |");
        Serial.println("|==============================================================|");
        Serial.print("| WiFi-Status:         ");
        if (WiFi.ready()) {
            Serial.println("ONLINE");
            toggle_runled();
            Serial.print("| Spark Cloud Status:  ");
            if (Spark.connected()) {
                Serial.println("ONLINE");
            } else {
                Serial.println("OFFLINE");
                WiFi.connect();
            }


            Serial.print("| IP:                  ");
            Serial.print(WiFi.localIP());
            Serial.print(" (");
            Serial.print(WiFi.subnetMask());
            Serial.println(")");
            Serial.print("| Gateway:             ");
            Serial.println(WiFi.gatewayIP());
            Serial.print("| WiFi Network (SSID): ");
            Serial.println(WiFi.SSID());
        } else {
            Serial.println("Offline");
        }

        if (WiFi.hasCredentials()) {
            // Serial.println("WiFi credentials stored:    ");
        } else {
            Serial.println("| No WiFi credentials stored!");
        }

        Serial.println("================================================================");

        Serial.print("| Listening to: ");


    #if ENABLE_TCP
        Serial.print("TCP, ");
    #endif

    #if ENABLE_UDP
        Serial.print("UDP, ");
    #endif

        Serial.print("Port: ");
        Serial.println(localPort, DEC);

        Serial.println("================================================================");
        Serial.println();


        Serial.println("=================================================================");
        Serial.println("|                  DMX STATUS ---- SPARK CORE                   |");
        Serial.println("|===============================================================|");
        Serial.println("|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|");

        uint16_t dmxChannel;
        for (uint8_t row = 0; row < DEBUG_DMX_ROWS; row++) {

            for (uint8_t col = 0; col < DEBUG_DMX_COLS; col++) {
                dmxChannel = row + DEBUG_DMX_ROWS * col + 1;
                if (dmxChannel > 512) {
                    Serial.print("|   -   ");
                } else {
                    Serial.print("|");
                    printDmxValue(dmxChannel);
                    Serial.print(" ");
                    printDmxValue(dmxData[dmxChannel - 1]);
                }
            }
            Serial.println("|");
        }

        Serial.println("|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|Ch.=Val|");
        Serial.println("|===============================================================|");

        Serial.println();
    }

    void loop() {
        uint16_t dmxChannel = 0;

        n++;
        if (n % 20 == 0) {
            if (!WiFi.ready()) {
                // Fast flash: WiFi not connected
                toggle_runled();
            } else if (n % 200 == 0) {
                // Slow flash: WiFi connected
                toggle_runled();

                if (!udpStarted && Udp.begin(localPort)) {
                    udpStarted = 1;
                }
                // Spark.process();
            }
            if (n % 10000 == 0) {
                printStatus();
                /*
                 * if (Spark.connected() == false) {
                    Spark.connect();
                }
                 */
            }
        }

        if (Serial.available()) {
            Serial.read();
            printStatus();
        }

    #if ENABLE_UDP
        checkDmxServer(TYPE_UDP);
    #endif

    #if ENABLE_TCP
        checkDmxServer(TYPE_TCP);
    #endif

        timeNow = millis();
        if (timeNow > timeLast + 1 || timeNow < timeLast) {
            for (dmxChannel = 0; dmxChannel < 512; dmxChannel++) {
                if (dmxData[dmxChannel] > dmxFade[dmxChannel]) {
                    dmxData[dmxChannel]--;
                } else if (dmxData[dmxChannel] < dmxFade[dmxChannel]) {
                    dmxData[dmxChannel]++;
                }
            }

            timeLast = timeNow;
            RGB.color(dmxData[0] / 2, dmxData[1] / 2, dmxData[2] / 2);

            uint8_t pin = 0;

            for (dmxChannel = 1; dmxChannel <= 16; dmxChannel++) {
                pin = dmxChannel - 1;

                // D0-7 = 0-7, A0-7 = 10-17
                if (pin > 7) pin = pin + 2;

                switch (pin) {
                    case D0:
                    case D1:
                    case A0:
                    case A1:
                    case A4:
                    case A5:
                    case A6:
                    case A7:
                        analogWrite(pin, dmxData[dmxChannel - 1]);
                        break;

                        // case A3:
                    case D2:
                    case D3:
                    case D4:
                    case D5:
                    case D6:
                    case D7:
                        // Write digital Values instantly (ignoring fade)
                        digitalWrite(pin, (dmxFade[dmxChannel - 1] >= 128));
                        break;
                }
            }
        }
        delay(1);
    }

#if ENABLE_DMX
extern "C" void Wiring_TIM3_Interrupt_Handler_override()
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        /*
         * BEGIN Time critical code
         */
        
        // bring the TX pin under GPIO control
        // this has the added effect that the pin is low
        set_pa2_gpio();
        
        // set the DE and (not)RE lines high
        toggle_usart2_de_re();
        
        // 100µs break
        tim2_wait_usec(100);
        
        // bring the TX pin under UART control
        // this has the added effect that the pin is high
        set_pa2_uart();
        
        // 12µs MAB (Mark After Break)
        tim2_wait_usec(12);
        
        usart2_tx_and_wait(0);
        
        // transmit the DMX data
        for (uint16_t i = 0; i < 512; i++) {
            
            // transmit the byte with a blocking send
            usart2_tx_and_wait(dmxData[i]);
        }
        
        // wait one additional character time before releasing the line
        tim2_wait_usec(44);
        
        // release the line
        toggle_usart2_de_re();
        
        
        /*
         * END Time critical code
         */
        
        toggle_runled();
    }
}
#endif
