/*
 * This project is forked from msp430fr599x_eusci_spi_standard_master.c by Nima Eskandari and Ryan Meredith in TI MSP430 official codes
 * Source: https://dev.ti.com/tirex/explore/node?node=ACHFXHTiNG.iukEIl1BQ2w__IOGqZri__LATEST
 *
 * Interface MSP430FR5994 with Adafruit external FRAM by SPI
 * Datasheet: https://cdn-shop.adafruit.com/product-files/4719/4719_MB85RS4MT.pdf
 *
 * SPI_Master_Communicate is flexible and standard SPI protocol
 * Could be used to interface with different hardware
 *
 */

// MSP430FR5994 SPI Pins Configure
//
//                   MSP430FR5994
//                 -----------------
//            /|\ |             P1.0|-> Comms LED
//             |  |                 |
//             ---|RST          P1.4|-> Slave Reset (GPIO)
//                |                 |
//                |             P5.0|-> Data Out (UCB1SIMO)
//                |                 |
//       Button ->|P5.5         P5.1|<- Data In (UCB1SOMI)
//   Button LED <-|P1.1             |
//                |             P5.2|-> Serial Clock Out (UCB1CLK)
//                |                 |
//                |             P5.3|-> Slave Chip Select (GPIO)
//

#include <msp430.h>
#include <stdint.h>
#include <gpio.h>

#define SLAVE_CS_OUT        P5OUT
#define SLAVE_CS_DIR        P5DIR
#define SLAVE_CS_PIN        BIT3

#define SLAVE_RST_OUT       P1OUT
#define SLAVE_RST_DIR       P1DIR
#define SLAVE_RST_PIN       BIT4

#define MAX_BUFFER_SIZE     1050

/*
 * Adafruit external FRAM opcode
 */
#define FRAM_EN             0x06
#define FRAM_DIS            0x04
#define FRAM_WR             0x02
#define FRAM_RD             0x03

uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};

uint16_t RXByteCtr = 0;
uint16_t TXByteCtr = 0;
uint16_t RXBytePtr = 0;
uint16_t TXBytePtr = 0;

void SendUCB1Data(uint8_t val)
{
    while (!(UCB1IFG & UCTXIFG));              // USCI_B1 TX buffer ready?
    UCB1TXBUF = val;
}


void SPI_Master_Communicate(uint16_t rx_size, uint16_t tx_size){
    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);

    RXBytePtr = 0;
    TXBytePtr = 0;
    RXByteCtr = rx_size;
    TXByteCtr = tx_size;

    TXByteCtr --;
    SendUCB1Data(TransmitBuffer[TXBytePtr ++]);

    __bis_SR_register(CPUOFF + GIE);
    SLAVE_CS_OUT |= SLAVE_CS_PIN;
}


void initSPI()
{
    //Clock Polarity: The inactive state is high
    //MSB First, 8-bit, Master, 3-pin mode, Synchronous
    UCB1CTLW0 = UCSWRST;                            // **Put state machine in reset**
    UCB1CTLW0 |= UCCKPL | UCMSB | UCSYNC
                | UCMST | UCSSEL__SMCLK;            // 3-pin, 8-bit SPI Slave
    UCB1BRW = 0x20;                                 // Set up SPI CLK. Bit clock prescaler setting. divide the clock of UCSSEL__****.
                                                    // e.g. SMCLK = 16MHz, UCB1BRW = 0x20 (32), SPI = 16MHz / 32 = 500KHz
    UCB1CTLW0 &= ~UCSWRST;                          // **Initialize USCI state machine**
    UCB1IE |= UCRXIE;                               // Enable USCI0 RX interrupt
}


void initGPIO()
{
    // Configure SPI
    P5SEL0 |= BIT0 | BIT1 | BIT2;

    SLAVE_RST_DIR |= SLAVE_RST_PIN;
    SLAVE_RST_OUT |= SLAVE_RST_PIN;

    SLAVE_CS_DIR |= SLAVE_CS_PIN;
    SLAVE_CS_OUT |= SLAVE_CS_PIN;

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    CSCTL0_H = CSKEY_H;                             // Unlock CS registers
    CSCTL1 = DCOFSEL_0;                             // Set DCO to 1MHz

    // Set SMCLK = MCLK = DCO, ACLK = VLOCLK
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;

    // Per Device Errata set divider to 4 before changing frequency to
    // prevent out of spec operation from overshoot transient
    CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4;           // Set all corresponding clk sources to divide by 4 for errata
    CSCTL1 = DCOFSEL_4 | DCORSEL;                   // Set DCO to 16MHz

    // Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
    __delay_cycles(60);
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;           // Set all dividers to 1 for 16MHz operation
    CSCTL0_H = 0;                                   // Lock CS registers
}

void test_fram_write(){
    uint16_t j = 0;
    for (j = 0; j < 0x400; j ++){
        TransmitBuffer[4 + j] = 0xFF;
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN2);

    for (j = 0; j < 0x40; j ++){
        // FRAM Write Enable Opcode
        TransmitBuffer[0] = FRAM_EN;
        SPI_Master_Communicate(0, 1);
        // FRAM Write Opcode
        TransmitBuffer[0] = FRAM_WR;
        // FRAM Write Address 24 bits
        TransmitBuffer[1] = 0x0;
        TransmitBuffer[2] = 0x0;
        TransmitBuffer[3] = 0x0;
        SPI_Master_Communicate(0, 0x404);
        // FRAM Write Disable Opcode
        TransmitBuffer[0] = FRAM_DIS;
        SPI_Master_Communicate(0, 1);
    }

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN2);
    __delay_cycles(10000000);
}

void test_fram_read(){
    uint16_t j = 0;
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN2);

    for (j = 0; j < 0x40; j ++){
        // FRAM Read Opcode
        TransmitBuffer[0] = FRAM_RD;
        // FRAM Read Address 24 bits
        TransmitBuffer[1] = 0x0;
        TransmitBuffer[2] = 0x0;
        TransmitBuffer[3] = 0x0;
        SPI_Master_Communicate(0x400, 0x4);
    }

    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN2);
    __delay_cycles(10000000);
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;                       // Stop watchdog timer

    initClockTo16MHz();
    initGPIO();
    initSPI();

    SLAVE_RST_OUT &= ~SLAVE_RST_PIN;                // Now with SPI signals initialized,
    __delay_cycles(100000);
    SLAVE_RST_OUT |= SLAVE_RST_PIN;                 // reset slave
    __delay_cycles(100000);                         // Wait for slave to initialize

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN2);

    test_fram_write();
    test_fram_read();

    __no_operation();
    __bis_SR_register(LPM0_bits + GIE);
}


/* SPI Interrupt */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B1_VECTOR))) USCI_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t ucb1_rx_val = 0;
    switch(__even_in_range(UCB1IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
            ucb1_rx_val = UCB1RXBUF;
            UCB1IFG &= ~UCRXIFG;
            if (TXByteCtr){
                TXByteCtr --;
                SendUCB1Data(TransmitBuffer[TXBytePtr ++]);
            }
            else if (RXByteCtr){
                SendUCB1Data(0x00);
                RXByteCtr --;
                ReceiveBuffer[RXBytePtr ++] = ucb1_rx_val;
            }
            else{
                __bic_SR_register_on_exit(CPUOFF);
            }
            break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}
