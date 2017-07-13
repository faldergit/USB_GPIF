#pragma NOIV               // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:       FX2_to_extsyncFIFO.c
//   Contents:   Hooks required to implement FX2 GPIF to external sync. FIFO
//               interface using CY4265-15AC
//
//   Copyright (c) 2003 Cypress Semiconductor, Inc. All rights reserved
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"            // SYNCDELAY macro, see Section 15.14 of FX2 Tech.
                                // Ref. Manual for usage details.

#define EXTFIFONOTFULL  (GPIFREADYSTAT & bmBIT1)
#define EXTFIFONOTEMPTY (GPIFREADYSTAT & bmBIT0)
#define LED_ALL         (bmBIT0 | bmBIT1 | bmBIT2 | bmBIT3)

extern BOOL GotSUD;             // Received setup data flag
extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

BYTE Configuration;             // Current configuration
BYTE AlternateSetting;          // Alternate settings
static WORD xdata LED_Count = 0;
static BYTE xdata LED_Status = 0;
BOOL in_enable = FALSE;         // flag to enable IN transfers

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
// The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------
void LED_Off (BYTE LED_Mask);
void LED_On (BYTE LED_Mask);
void GpifInit ();

void GPIF_SingleByteWrite (BYTE gdata)
{
  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }

  XGPIFSGLDATLX = gdata;         // trigger GPIF Single Byte Write transaction
}

void GPIF_SingleByteRead (BYTE xdata *gdata)
{
  static BYTE g_data = 0x00;     // dummy variable

  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }

  // using register in XDATA space, dummy read
  g_data = XGPIFSGLDATLX;        // trigger GPIF Single Byte Read transaction

  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }

  // using register in XDATA space, 
  *gdata = XGPIFSGLDATLNOX;      // retrieve byte just read from ext. FIFO
}

void GPIF_SingleWordWrite( WORD gdata )
{
  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
    ;
  }

  // using registers in XDATA space
  XGPIFSGLDATH = gdata;              
  XGPIFSGLDATLX = gdata >> 8;         // trigger GPIF Single Word Write transaction
}

void GPIF_SingleWordRead( WORD xdata *gdata )
{
  static BYTE g_data = 0x00;     // dummy variable

  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }

  // using register in XDATA space
  g_data = XGPIFSGLDATLX;        // dummy read to trigger GPIF 
                                 // Single Word Read transaction

  while( !( GPIFTRIG & 0x80 ) )  // poll GPIFTRIG.7 Done bit
  {
     ;
  }

  // using register(s) in XDATA space, retrieve word just read from ext. FIFO
  *gdata = ( ( WORD )XGPIFSGLDATLNOX << 8 ) | ( WORD )XGPIFSGLDATH;
}


void TD_Init(void)             // Called once at startup
{
  // set the CPU clock to 48MHz
  CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1);
  SYNCDELAY;
  
  EP2CFG = 0xA0;     // EP2OUT, bulk, size 512, 4x buffered
  SYNCDELAY;                    
  EP4CFG = 0x00;     // EP4 not valid
  SYNCDELAY;                    
  EP6CFG = 0xE0;     // EP6IN, bulk, size 512, 4x buffered
  SYNCDELAY;                    
  EP8CFG = 0x00;     // EP8 not valid
  SYNCDELAY;
  
  EP2FIFOCFG = 0x01; // manual mode, disable PKTEND zero length send, word ops
  SYNCDELAY;
  EP6FIFOCFG = 0x01; // manual mode, disable PKTEND zero length send, word ops
  SYNCDELAY;

  FIFORESET = 0x80;  // set NAKALL bit to NAK all transfers from host
  SYNCDELAY;
  FIFORESET = 0x02;  // reset EP2 FIFO
  SYNCDELAY;
  FIFORESET = 0x06;  // reset EP6 FIFO
  SYNCDELAY;
  FIFORESET = 0x00;  // clear NAKALL bit to resume normal operation
  SYNCDELAY;

  // out endpoints do not come up armed
  // since EP2OUT is quad buffered we must write dummy byte counts four times
                  
  EP2BCL = 0x80;     // arm EP2OUT by writing byte count w/skip.
  SYNCDELAY;                    
  EP2BCL = 0x80;
  SYNCDELAY;                    
  EP2BCL = 0x80;
  SYNCDELAY;
  EP2BCL = 0x80;
  SYNCDELAY;

  GpifInit (); // initialize GPIF registers

  // reset the external FIFO

  OEA |= 0x04;     // turn on PA2 as output pin
  IOA |= 0x04;     // pull PA2 high initially   
  IOA &= 0xFB;     // bring PA2 low
  EZUSB_Delay (1); // keep PA2 low for ~1ms, more than enough time
  IOA |= 0x04;     // bring PA2 high   

  /******自行添加代码*****/
  OEA|=0x80;  //set FX_SLCS PA7 as output pin
  IOA|=0x80; //pull PA2 High  open FPGA's power

  
  
}

void TD_Poll(void)
{
  WORD xdata  i;            // loop index
  WORD xdata  *Source;      // ptr to source buffer
  WORD xdata  *Destination; // ptr to destination buffer
  WORD xdata  Tcount;       // transaction counter

  if(!(EP2468STAT & bmEP2EMPTY) && (EXTFIFONOTFULL))
  {
    // if host sent data to EP2OUT AND external FIFO is not full,

    Tcount = (EP2BCH << 8) + EP2BCL; // load transaction count with EP2 byte count
    Tcount /= 2;                     // divide by 2 for word wide transaction
    Source = (WORD *)(&EP2FIFOBUF);
    for( i = 0x0000; i < Tcount; i++ )
    {
      // transfer data from EP2OUT buffer to external FIFO
      GPIF_SingleWordWrite (*Source);
      Source++;
    }
    EP2BCL = 0x80; // re-arm EP2OUT
  }
    
  if(in_enable) // if IN transfers are enabled,
  {
    if(!(EP2468STAT & bmEP6FULL) && (EXTFIFONOTEMPTY))
    {
      // if EP6IN is not full AND there is data in the external FIFO,

      Destination = (WORD *)(&EP6FIFOBUF);
      for( i = 0x0000; i < Tcount; i++ )
      {
        // transfer data from external FIFO to EP6IN buffer
        GPIF_SingleWordRead (Destination);
        Destination++;
      }
	  Tcount *= 2;          // multiply by 2 to obtain byte count value    
      EP6BCH = MSB(Tcount);  
      SYNCDELAY;  
      EP6BCL = LSB(Tcount); // arm EP6IN to send data to the host
      SYNCDELAY;                    
    }
  }

  // blink LED0 to indicate firmware is running

  if (++LED_Count == 10000)
  {
    if (LED_Status)
    {
      LED_Off (bmBIT0);
      LED_Status = 0;
    }
    else
    {
      LED_On (bmBIT0);
      LED_Status = 1;
    }
    LED_Count = 0;
  }

}

BOOL TD_Suspend(void)          // Called before the device goes into suspend mode
{
   return(TRUE);
}

BOOL TD_Resume(void)          // Called after the device resumes
{
   return(TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}

BOOL DR_SetConfiguration(void)   // Called when a Set Configuration command is received
{
   Configuration = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{
   AlternateSetting = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}

#define VX_B2 0xB2 // reset external FIFO
#define VX_B3 0xB3 // enable IN transfers
#define VX_B4 0xB4 // disable IN transfers

BOOL DR_VendorCmnd(void)
{
  
  switch (SETUPDAT[1])
  {
    case VX_B2:
    { 
      // reset the external FIFO

      OEA |= 0x04;     // turn on PA2 as output pin
      IOA |= 0x04;     // pull PA2 high initially   
      IOA &= 0xFB;     // bring PA2 low
      EZUSB_Delay (1); // keep PA2 low for ~1ms, more than enough time
      IOA |= 0x04;     // bring PA2 high

      *EP0BUF = VX_B2;
	  EP0BCH = 0;
	  EP0BCL = 1;                   // Arm endpoint with # bytes to transfer
	  EP0CS |= bmHSNAK;             // Acknowledge handshake phase of device request
      break;
    }
	case VX_B3: // enable IN transfers
	{
	  in_enable = TRUE;

      *EP0BUF = VX_B3;
  	  EP0BCH = 0;
	  EP0BCL = 1;
	  EP0CS |= bmHSNAK;
	  break;
    }
	case VX_B4: // disable IN transfers
	{
	  in_enable = FALSE;

      *EP0BUF = VX_B4;
  	  EP0BCH = 0;
	  EP0BCL = 1;
	  EP0CS |= bmHSNAK;
	  break;
    }
    default:
        return(TRUE);
  }

  return(FALSE);
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) interrupt 0
{
   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
   // whenever we get a USB reset, we should revert to full speed mode
   pConfigDscr = pFullSpeedConfigDscr;
   ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
   pOtherConfigDscr = pHighSpeedConfigDscr;
   ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
   Sleep = TRUE;
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
      pOtherConfigDscr = pFullSpeedConfigDscr;
      ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;
   }

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}
void ISR_Ep2inout(void) interrupt 0
{
}
void ISR_Ep4inout(void) interrupt 0
{
}
void ISR_Ep6inout(void) interrupt 0
{
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}

// ...debug LEDs: accessed via movx reads only ( through CPLD )
// it may be worth noting here that the default monitor loads at 0xC000
xdata volatile const BYTE LED0_ON  _at_ 0x8000;
xdata volatile const BYTE LED0_OFF _at_ 0x8100;
xdata volatile const BYTE LED1_ON  _at_ 0x9000;
xdata volatile const BYTE LED1_OFF _at_ 0x9100;
xdata volatile const BYTE LED2_ON  _at_ 0xA000;
xdata volatile const BYTE LED2_OFF _at_ 0xA100;
xdata volatile const BYTE LED3_ON  _at_ 0xB000;
xdata volatile const BYTE LED3_OFF _at_ 0xB100;
// use this global variable when (de)asserting debug LEDs...
BYTE xdata ledX_rdvar = 0x00;
BYTE xdata LED_State = 0;
void LED_Off (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_OFF;
		LED_State &= ~bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_OFF;
		LED_State &= ~bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_OFF;
		LED_State &= ~bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_OFF;
		LED_State &= ~bmBIT3;
	}
}

void LED_On (BYTE LED_Mask)
{
	if (LED_Mask & bmBIT0)
	{
		ledX_rdvar = LED0_ON;
		LED_State |= bmBIT0;
	}
	if (LED_Mask & bmBIT1)
	{
		ledX_rdvar = LED1_ON;
		LED_State |= bmBIT1;
	}
	if (LED_Mask & bmBIT2)
	{
		ledX_rdvar = LED2_ON;
		LED_State |= bmBIT2;
	}
	if (LED_Mask & bmBIT3)
	{
		ledX_rdvar = LED3_ON;
		LED_State |= bmBIT3;
	}
}

