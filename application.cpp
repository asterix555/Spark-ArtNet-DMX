#include "application.h"

/*
 To receive ArtNet packages you need to modify one line in ../inc/spark_wiring_udp.h from
    #define RX_BUF_MAX_SIZE	512
 to
    #define RX_BUF_MAX_SIZE	640
*/


/*
 Used Spark Core Pins

 TX (PA2)   DI      (USART2)
 A2 (PA4)   DE/!RE  (Not really necessary if you only want to continuously send DMX)
 D7 (PA13)  Blue LED
 
*/

#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x) (LOW_BYTE & x)
#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) )


#define toggle_runled() GPIOA->ODR ^= (1 << 13) // Toggle Sparks blue LED

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

// Universe
short selected_universe = 1;

// Buffers
const int MAX_BUFFER_UDP = 640;
char packetBuffer[MAX_BUFFER_UDP]; // Incoming UDP data buffer

uint8_t dmx_data[512]; // Index 0 is DMX channel 1

// ArtNet parameters
unsigned int localPort = 5568; // "When multicast addressing is used, the UDP destination Port shall be set to the standard ACN-SDT multicast port (5568)"
const int max_packet_size = 640;

const char E131Header[] = { 0x00, 0x10, 0x00, 0x00, 0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};

UDP Udp;

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

void setup()
{
    // USB Serial for debugging
    Serial.begin(9600);
    
    // Setup UDP
    Udp.begin(localPort);
    
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    // DMX
    setupDMX();

    // Control the RGB LED
    RGB.control(true);
}

void loop()
{
    int packetSize = Udp.parsePacket();

    if(packetSize >= 638) // Check size to avoid unnecessary checks
    {
        Udp.read(packetBuffer, MAX_BUFFER_UDP); // Read data into buffer
        
        // Read header
        boolean match_artnet = 1;
        for (uint8_t i = 0; i<(sizeof E131Header); i++)
        {
            // if not corresponding, this is not an artnet packet, so we stop reading
            if(char(packetBuffer[i]) != E131Header[i])
            {
                match_artnet = 0;
                break;
            }
        }
        
        if(match_artnet == 1){
            
            uint32_t framing_vector = (packetBuffer[40] << 24) | (packetBuffer[41] << 16) | (packetBuffer[42] << 8) | packetBuffer[43];
            
            // Check if this is E1.31 data
            if(framing_vector != 0x00000002){
                match_artnet = 0;
            }
        }
        
        // if it's an artnet header
        if(match_artnet == 1)
        {
            short incoming_universe = packetBuffer[113] << 8 | packetBuffer[114];
            
            // Check universe
            if(incoming_universe == selected_universe)
            {
                for(int i = 0; i < 512; i++){
                    dmx_data[i] = byte(packetBuffer[126 + i]);
                }
                
                // Set RGB LED with channel 1, 2 and 3
                RGB.color(dmx_data[0], dmx_data[1], dmx_data[2]);
            }
        }
    }
}

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
            usart2_tx_and_wait(dmx_data[i]);
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