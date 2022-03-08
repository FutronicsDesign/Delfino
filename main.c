/* Author: Dave * Blink a light, don't forget your EALLOWs */

#include <F28x_Project.h>
#include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions
#include "F2837xD_device.h"        // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"      // F2837xD Examples Include File
#include "stdint.h"
#include "pl455.h"
// Defines
//
#define CPU_FREQ        60E6
#define LSPCLK_FREQ     CPU_FREQ/4
#define SCI_FREQ        100E3
#define SCI_PRD         (LSPCLK_FREQ/(SCI_FREQ*8))-1


int nSent, nRead = 0;

//
// Globals
//
uint16_t sdataA[2];    // Send data for SCI-A
uint16_t rdataA[2];    // Received data for SCI-A
uint16_t rdata_pointA; // Used for checking the received data
BYTE  bFrame[132];
#define nDev_ID 0

//
// Function Prototypes
//
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
void CommClear(void);
void scia_fifo_init(void);
void error(void);

int main(void)
{
    uint16_t i;


 //
 // Step 1. Initialize System Control:
 // PLL, WatchDog, enable Peripheral Clocks
 // This example function is found in the F2837xD_SysCtrl.c file.
 //
    InitSysCtrl();

 //
 // Step 2. Initialize GPIO:
 // This example function is found in the F2837xD_Gpio.c file and
 // illustrates how to set the GPIO to it's default state.
 //
    InitGpio();

 //
 // For this example, only init the pins for the SCI-A port.
 //  GPIO_SetupPinMux() - Sets the GPxMUX1/2 and GPyMUX1/2 register bits
 //  GPIO_SetupPinOptions() - Sets the direction and configuration of the GPIOS
 // These functions are found in the F2837xD_Gpio.c file.
 //
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

 //
 // Step 3. Clear all interrupts and initialize PIE vector table:
 // Disable CPU interrupts
 //
    DINT;

 //
 // Initialize PIE control registers to their default state.
 // The default state is all PIE interrupts disabled and flags
 // are cleared.
 // This function is found in the F2837xD_PieCtrl.c file.
 //
    InitPieCtrl();

 //
 // Disable CPU interrupts and clear all CPU interrupt flags:
 //
    IER = 0x0000;
    IFR = 0x0000;

 //
 // Initialize the PIE vector table with pointers to the shell Interrupt
 // Service Routines (ISR).
 // This will populate the entire table, even if the interrupt
 // is not used in this example.  This is useful for debug purposes.
 // The shell ISR routines are found in F2837xD_DefaultIsr.c.
 // This function is found in F2837xD_PieVect.c.
 //
    InitPieVectTable();

 //
 // Interrupts that are used in this example are re-mapped to
 // ISR functions found within this file.
 //
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
    PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

 //
 // Step 4. Initialize the Device Peripherals:
 //
    scia_fifo_init();  // Init SCI-A
 //   CommClear();
 //
 // Step 5. User specific code, enable interrupts:
 //
 // Init send data.  After each transmission this data
 // will be updated for the next transmission
 //
    for(i = 0; i<2; i++)
    {
       sdataA[i] = i;
    }

//    rdata_pointA = sdataA[0];

 //
 // Enable interrupts required for this example
 //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2
    IER = 0x100;                         // Enable CPU INT
    EINT;

//    printf("BMS INITIALIZ...........\n");
//  Wake devices ID 0
//  The wake tone will awaken any device that is already in shutdown and the pwrdown will shutdown any device that is already awake.
//  The least number of times to sequence wake and pwrdown will be half the number of
//  Boards to cover the worst case combination of boards already awake or shutdown.
//    nSent = WriteReg(nDev_ID, 12, 0x40, 1, FRMWRT_SGL_NR);      // send out broadcast pwrdown command
    delayms(5); //~5ms
    WakePL455();
    delayms(5); //~5ms
//  Mask Customer Checksum Fault bit
    nSent = WriteReg(nDev_ID, 107, 0x8000, 2, FRMWRT_SGL_NR);   // clear all fault summary flags
//  Clear all faults
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);    // clear all fault summary flags
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR);      // clear fault flags in the system status register
//  Set addresses for all boards in daisy-chain (section 1.2.3)
    nSent = WriteReg(nDev_ID, 10, nDev_ID, 1, FRMWRT_SGL_NR);   // send address to each board
//  Enable/Disable communication interfaces on all/single boards in the stack (section 1.2.1)
//    nSent = WriteReg(nDev_ID, 16, 0x10F8, 2, FRMWRT_SGL_NR);    // set communications baud rate and enable all interfaces on all boards in stack
//    nSent = WriteReg(nDev_ID, 16, 0x1020, 2, FRMWRT_SGL_NR);    // Disable High side receiver on differential (1.2.5)
//    nSent = WriteReg(nDev_ID, 16, 0x10C0, 2, FRMWRT_SGL_NR);    // Disable low side receiver on differential (1.2.6)
    nSent = WriteReg(nDev_ID, 16, 0x1080, 2, FRMWRT_SGL_NR);    // only baud rate and uart on differential and all disable (1.2.6)
// Clear all faults (section 1.2.7)
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_ALL_NR);    // clear all fault summary flags
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_ALL_NR);      // clear fault flags in the system status register
    delayms(10);
// Configure AFE best setting
    nSent = WriteReg(0, 61, 0x00, 1, FRMWRT_ALL_NR);            // set 0 initial delay
// Configure cell voltage and internal temp sample period
    nSent = WriteReg(0, 62, 0xB4, 1, FRMWRT_ALL_NR);            // set (CC) 100us cell and 100us temp ADC sample period
// Configure AUX voltage sample period
    nSent = WriteReg(0, 63, 0x44444444, 4, FRMWRT_ALL_NR);      // set 12.6us AUX sample period
// Configure the oversampling rate
    nSent = WriteReg(0, 7, 0x7B, 1, FRMWRT_ALL_NR);             // set 8x oversampling, stay on channel for oversample and 12.6us oversample period for cell and AUX
// Set AFE_PCTL
    nSent = WriteReg(0, 15, 0x80, 1, FRMWRT_ALL_NR);            // set AFE_PCTL bit to on (only enable AFE when sampling)
//// Configure AFE to Recomended setting
  // {
//    nSent = WriteReg(0, 61, 0x00, 1, FRMWRT_ALL_NR);            // set 0 initial delay
//// Configure cell voltage and internal temp sample period
//    nSent = WriteReg(0, 62, 0xB4, 1, FRMWRT_ALL_NR);            // set 4.13us cell and 12.6us temp ADC sample period
//// Configure AUX voltage sample period
//    nSent = WriteReg(0, 63, 0x44444444, 4, FRMWRT_ALL_NR);      // set 12.6us AUX sample period
//// Configure the oversampling rate
//    nSent = WriteReg(0, 7, 0x7B, 1, FRMWRT_ALL_NR);             // set 0 oversampling
//// Set AFE_PCTL
//    nSent = WriteReg(0, 15, 0x80, 1, FRMWRT_ALL_NR);            // set AFE_PCTL bit to on (only enable AFE when sampling)
  // }
// Clear and check faults (section 2.2.4)
    nSent = WriteReg(nDev_ID, 81, 0x38, 1, FRMWRT_SGL_NR);      // clear fault flags in the system status register
    nSent = WriteReg(nDev_ID, 82, 0xFFC0, 2, FRMWRT_SGL_NR);    // clear all fault summary flags
// Select number of cells and Aux channels to sample (section 2.2.5.1)
    nSent = WriteReg(nDev_ID, 13, 0x08, 1, FRMWRT_SGL_NR);      // set number of cells to 8
    nSent = WriteReg(nDev_ID, 3, 0x00FFFF42, 4, FRMWRT_SGL_NR); // select 8 cell, 8 AUX channels, internal analog die temperature and vpack
// Developed by Gunit..........
    nSent = WriteReg(0, 67, 0x4900, 2, FRMWRT_ALL_NR);          // set vm sampling period
    nSent = WriteReg(0, 30, 0x0001, 2, FRMWRT_ALL_NR);          // enable vm
// Set cell over-voltage and cell under-voltage thresholds on a single board (section 2.2.6.1)
    nSent = WriteReg(nDev_ID, 144, 0xD998, 2, FRMWRT_SGL_NR);   // set OV threshold = 4.2500V
    nSent = WriteReg(nDev_ID, 142, 0x8A3C, 2, FRMWRT_SGL_NR);   // set UV threshold = 2.7000V
// Configure GPIO pin direction and set new pin values (section 5.2.1)
    nSent = WriteReg(nDev_ID, 123, 0x00, 1, FRMWRT_SGL_NR);     // turn off all GPIO pull downs
    nSent = WriteReg(nDev_ID, 122, 0x00, 1, FRMWRT_SGL_NR);     // turn off all GPIO pull ups
    nSent = WriteReg(nDev_ID, 120, 0x3F, 1, FRMWRT_SGL_NR);     // set GPIO[5:0] to output direction
    nSent = WriteReg(nDev_ID, 122, 0x3F, 1, FRMWRT_SGL_NR);     // turn on all GPIO pull ups
    nSent = WriteReg(nDev_ID, 121, 0x00, 1, FRMWRT_SGL_NR);     // set GPIO outputs (as 0)
    delayms(2500);
    nSent = WriteReg(nDev_ID, 2, 0x01, 1, FRMWRT_SGL_R);        // send sync sample command
    //nSent = WaitRespFrame(bFrame, 39, 0);                       // 1+16(8 cell voltage) + 16(8temp) bytes data + packet header + CRC(2), 0ms timeout

//    printf("Battery Initial Status:\n");
 //   adcRead();
 //   iPack = getCellCurrent(c);
 //   vPack = getCellVoltage(bFrame,c);
 //   TempPack = getCellTemp(bFrame,c);
 //   zAvg = getInitialSOC(c);
 //   PCap = calculateCellCapacity(c, deltaT);
 //   Print();
 //   showdata4(c,vPack,iPack,zAvg,TempPack, PCap);
   // spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Voltage_Master);
    delayms(250);
 //   spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Soc_Master);
    delayms(250);
   // spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Aux_Master);
    delayms(250);
 //   CAN_Cell(bFrame);
 //   CAN_Pack(vPack, iPack, zAvg, TempPack);
    delayms(250);

 //   ContactorON();
//    printf("\n");
    delayms(10);
    while(1){
   //     gioToggleBit(gioPORTA,2);
     //   Floating(bFrame, c, iref, iPack);
     //   spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Voltage_Master);
        delayms(250);
     //   spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Soc_Master);
        delayms(250);
     //   spiTransmitData(spiREG1, &dataconfig1_t, SPI_DATA_LENGTH, TX_Aux_Master);
        delayms(250);
     //   CAN_Cell(bFrame);
     //   CAN_Pack(vPack, iPack, zAvg, TempPack);
        delayms(250);
    }


}

interrupt void sciaTxFifoIsr(void)
{
   uint16_t i;

    for(i=0; i< 2; i++)
    {
       SciaRegs.SCITXBUF.all=sdataA[i];  // Send data
    }

    for(i=0; i< 2; i++)                  // Increment send data for next cycle
    {
       sdataA[i] = (sdataA[i]+1) & 0x00FF;
    }

    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ACK
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
interrupt void sciaRxFifoIsr(void)
{
    uint16_t i;

    for(i=0;i<2;i++)
    {
       rdataA[i]=SciaRegs.SCIRXBUF.all;  // Read data
    }

    for(i=0;i<2;i++)                     // Check received data
    {
       if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) )
       {
    //       error();
       }
    }

    rdata_pointA = (rdata_pointA+1) & 0x00FF;

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}


void scia_fifo_init()
{
   SciaRegs.SCICCR.all = 0x0007;      // 1 stop bit,  No loopback
                                      // No parity,8 char bits,
                                      // async mode, idle-line protocol
   SciaRegs.SCICTL1.all = 0x0003;     // enable TX, RX, internal SCICLK,
                                      // Disable RX ERR, SLEEP, TXWAKE
   SciaRegs.SCICTL2.bit.TXINTENA = 1;
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
   SciaRegs.SCIHBAUD.all = 0x0000;
   SciaRegs.SCILBAUD.all = SCI_PRD;
   SciaRegs.SCICCR.bit.LOOPBKENA = 1; // Enable loop back
   SciaRegs.SCIFFTX.all = 0xC022;
   SciaRegs.SCIFFRX.all = 0x0022;
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

