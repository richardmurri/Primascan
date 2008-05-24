/*******************************************************************************
 *  Primascan.c
 *
 *  Purpose: To provide a driver that can be used on a linux operating system
 *           that will be able to scan from the Primax Colorado 2400u scanner.
 *           This driver was designed to work with the SANE Api.  It has been
 *           modified into a standalone driver so that one does not have to
 *           install SANE and all of its applications before use.
 *
 *  Author:  Richard Murri
 *  Date:    Nov 16, 2005
 *
   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.
 ******************************************************************************/




/*******************************************************************************
 *  Most of our transfer data is contained in primascan.h
 *  There are several structures:
 *  scannerSetup              - for initialization of the scanner
 *  setupBlack and setupColor - for getting the scanner ready for a particular
 *                              scan.
 *  calibrationWrite          - for a specific calibration write to the scanner
 *  scanBlack and scanColor   - for performing a particular scan
 *  finalize                  - for finalizing the scanner
 ******************************************************************************/
#include "primascan.h"
#include <usb.h>
#include <stdio.h>




/*******************************************************************************
 * Since SANE doesn't give us a way to store certain information we must
 * store them in static global variables.  
 *
 * deviceHandle - A pointer to the open libusb device
 *
 * isDeviceOpen - (0) is no, anything else is yes
 *
 * dpiValue -     The current dpi value.  Only 100 (color) and 200 (black/white)
 *                are allowed.
 *
 * largeBuffer -  Information that is read during a scan is kept in 
 *                largeBuffer.  When large amounts of memory are obtained and 
 *     	          released from the heap, errors occur.  This buffer prevents us 
 *                from needing to allocate memory from the heap for every scan 
 *                sequence.
 ******************************************************************************/
static usb_dev_handle *deviceHandle = NULL;
static int isDeviceOpen = 0;
static int dpiValue = 100;
static char largeBuffer[0xffff];




/*******************************************************************************
 *  Non-SANE functions
 *  ------------------
 *
 *  detectDevice() -   If the scanner is attached, it will return 1.  If not, it
 *                     will return 0.
 *
 *  controlTransfer()- When we need to send a control transfer to the scanner
 *                     we need information about the requestType, request,
 *                     value, index, and size fields required by the USB
 *                     specification.  We pass in an integer pointer where
 *                     data[0] = requestType;
 *                     data[1] = request;
 *                     data[3] + data[2] = value;
 *                     data[5] + data[4] = index;
 *                     data[7] + data[6] = size;
 *                     Where the '+' operator means concatenation.  This format
 *                     follows the format of sniffusb.
 *
 *  reapeatedControlTransfer() - This performs a control transfer multiple
 *                     times until the desired value is given by
 *                     the scanner.  This is used to wait until the
 *                     scanner is ready. The parameters are the same
 *                     except there is an additional integer value
 *                     that represents that value that the scan is
 *                     waiting for in data[8].
 *
 *  writeBulk0s() -    Often the scanner requires a bulk write of nothing but 
 *                     '0's.  This function will write that to the scanner.
 *                     data[0] = 0xff
 *                     data[1] = The endpoint for the bulk write
 *                     data[3] + data[2] = The size of the write
 *
 *  calibrationWrite() - A special calibration data set will need to be written
 *                     to the scanner three times during a scan.  That data
 *                     is represented by the variable calibWrite.  The
 *                     parameter is a pointer to calibWrite.
 *
 *  calibrate() -      A certain sequence of calibration needs to be generated
 *                     at certain times in the scan.  This function generates
 *                     that sequence and sends it to the scanner.
 *
 *  finalizeScanner()- After reading the scanned data we need to perform a
 *                     few more operations.  This function will run through
 *                     those.
 ******************************************************************************/
int detectDevice ();
int controlTransfer (int *data);
int repeatedControlTransfer (int *data);
int writeBulk0s (int *data);
int calibrationWrite (int *data);
int calibrate ();
int finalizeScanner ();




/*******************************************************************************
 *  SANE functions (as defined in the SANE API)
 *  ---------------
 *
 *  NOTE:              Many of the SANE functions have parameters.  They are
 *                     not needed for this stand alone program and have
 *                     been left out.
 *
 *  sane_init() -      Initializes  the driver
 *
 *  sane_getdevices()- Detects any devices and returns a handle to represent
 *                     them.  This function is only needed for compatibility
 *                     and its effectiveness has been removed for this 
 *                     application.
 *
 *  sane_open() -      Open the USB scanner if there is one attached.  It will
 *                     change the environment variable isDeviceOpen to 1 if
 *                     successful.
 *
 *  sane_close() -     Will close any open USB scanner.  Sets isDeviceOpen to 0.
 *
 *  sane_exit() -      Makes sure everything is closed before exiting
 *
 *
 *  sane_get_optiondescriptor () -  These functions are only included to show
 *  sane_controloption () -         that corresponding SANE functions are
 *  sane_getparameteres () -        included in the SANE driver
 *  
 *
 *  sane_start() -     Runs through all of the configuration needed to start
 *                     the scan.  There are three phases.
 *                     - Initialize Scanner - Get the scanner ready to be set 
 *                               up. It uses the static variable 'scannerSetup'
 *                                            
 *                     - Scanner Setup - This is different for color and text
 *                               scans.  It uses 'setupBlack' or 'setupColor'
 *                               depending on the mode.
 *
 *                     - Calibration - Both color and text scans perform the
 *                               same calibration data.
 *
 *  sane_read ()       - Performs the scan and reads data into the program.
 *                       -> *buf - A pointer to a buffer at least max_len 
 *                               bytes large
 *                       -> max_len - The buffer is at least this large
 *                       -> *len - A pointer to let the program know how 
 *                               much data is available.
 *
 ******************************************************************************/
void sane_init ()
{
  /* Initialize usb and find usb devices */

  usb_init ();
  usb_find_busses ();
  usb_find_devices ();
}

void sane_getdevices ()
{
  /* Check if the device is attached */
  struct usb_device dev;
  detectDevice (&dev);
}

void sane_open ()
{

  struct usb_device dev;

  /* If the device is attached */
  if (detectDevice (&dev))
  {
    /* Open device */
    deviceHandle = usb_open (&dev);

    int status1;
    int status2;
    int status3;

    /* Configure the Device */
    status1 = usb_set_configuration (deviceHandle, 1);
    status2 = usb_claim_interface (deviceHandle, 0);
    status3 = usb_set_altinterface (deviceHandle, 0);

    /* If any of the configuration fails */
    if ((deviceHandle == NULL) || (status1 < 0) ||
	(status2 < 0) || (status3 < 0))
    {
      fprintf (stderr, "Problem opening device\n");
      exit (1);
    }

    isDeviceOpen = 1;
    return;
  }

  /* Device is not attached */
  fprintf (stderr, "Device could not be found\n");
  exit (1);

}

void sane_close ()
{
  /* Close any open device */
  if (isDeviceOpen)
  {
    usb_reset (deviceHandle);
    isDeviceOpen = 0;
  }
}

void sane_exit ()
{
  sane_close ();
}

void sane_get_optiondescriptor ()
{
}

void sane_controloption ()
{
}

void sane_getparameteres ()
{
}

void sane_start ()
{
  int i;
  int result;

  /*********************************
   * Initialize scanner 
   ********************************/
  for (i = 0; i < scannerSetupSize; i++)
  {
    result = controlTransfer (scannerSetup[i]);

    if (result != 1)
    {
      fprintf (stderr, "******************\n");
      fprintf (stderr, "Something went wrong\n");
      fprintf (stderr, "Result not equal to 1\n");
      fprintf (stderr, "Error in 'Initialize Scanner'\n");
      fprintf (stderr, "Urb %d and setup line %d\n", i, i);
      fprintf (stderr, "******************\n");
      exit (1);
    }
  }


  /*******************************
   * Scanner Setup
   ******************************/
  int *typePtr;
  int typeSize;

  /* Setup is different for black or for color */
  if (dpiValue == 200)
  {
    typePtr = setupBlack[0];
    typeSize = setupBlackSize;
  }
  else
  {
    typePtr = setupColor[0];
    typeSize = setupColorSize;
  }


  for (i = 0; i < typeSize; i++)
  {

    /*
     * There are several types of transfers:               
     *
     *   -Bulk Read                  - represented by 0xfa                  
     *   -Repeated Control Transfers - represented by 0xfb 
     *   -Write Bulk 0s              - represented by 0xff             
     *   -Anything else              - regular Control Transfer         
     */

    if (*(typePtr + (i * 16)) == 0xfa)
    {
      /* Bulk Read */
      result = bulkRead (typePtr + (i * 16));
    }
    else if (*(typePtr + (i * 16)) == 0xfb)
    {
      /* Repeat Command */
      result = repeatedControlTransfer (typePtr + (i * 16));
    }
    else if (*(typePtr + (i * 16)) == 0xff)
    {
      /* Bulk write 0s */
      result = writeBulk0s (typePtr + (i * 16));
    }
    else
    {
      /* Normal Control Transfer */
      result = controlTransfer (typePtr + (i * 16));
    }

    if (result != 1)
    {
      fprintf (stderr, "******************\n");
      fprintf (stderr, "Something went wrong\n");
      fprintf (stderr, "Result not equal to 1\n");
      fprintf (stderr, "Error in 'Scanner Setup'\n");
      fprintf (stderr, "Urb %d and setup line %d\n", i + 78, i);
      fprintf (stderr, "******************\n");
      exit (1);
    }
  }


  /****************************
   * Scanner Calibration
   ***************************/

  for (i = 0; i < calibrationSize; i++)
  {
    if (calibration[i][0] == 0xfc)
    {
      /* If we need to do the special calibration */
      result = calibrationWrite (calibration[i]);
    }
    else if (calibration[i][0] == 0xfd)
    {
      /* If we need a calculated calibration */
      result = calibrate ();
    }
    else
    {
      /* Normal control transfer */
      result = controlTransfer (calibration[i]);
    }

    /* If there's a problem anywhere */
    if (result != 1)
    {
      fprintf (stderr, "******************\n");
      fprintf (stderr, "Something went wrong\n");
      fprintf (stderr, "Result not equal to 1\n");
      fprintf (stderr, "Error in 'Scanner Calibration'\n");

      if (dpiValue == 200)
	fprintf (stderr, "Urb %d and setup line %d\n", i + 905, i);
      else
	fprintf (stderr, "Urb %d and setup line %d\n", i + 1056, i);

      fprintf (stderr, "******************\n");
      exit (1);
    }
  }

/* The scanner is now ready for the actual scan */
}

static tempVar = 0;


void sane_read (char *buf, int max_len, int *len)
{
  /*
   * We want these variables to retain value between function calls    
   *   In essence we need to do this read across several function calls
   *   as if it was only one call                                      
   */

  static int dataAvailable = 0;	/* We currently have no data available  */
  static int whereInBuffer = 0;	/* We must point at beginning of buffer */
  static int i = 0;



  int *typePtr;
  int typeSize;
  int result;

  /* The scan is different for black or for color */
  if (dpiValue == 200)
  {
    typePtr = scanBlack[0];
    typeSize = scanBlackSize;
  }
  else
  {
    typePtr = scanColor[0];
    typeSize = scanColorSize;
  }

  /* Don't reinitialize 'i', we need it to be static */
  for (; i < typeSize; i++)
  {
    /*
     * If we didn't finish giving all of our data to the function 
     * last time because the buffer was too small, do it now.     
     */

    if (dataAvailable > 0)
    {
      if (dataAvailable < max_len)
      {
	int j;

	/* copy available data to buffer */
	for (j = 0; j < dataAvailable; ++j)
	  buf[j] = largeBuffer[j + whereInBuffer];

	*len = dataAvailable;
	dataAvailable = 0;
	whereInBuffer = 0;
      }
      else
      {
	int j;

	/* copy available data up to max_len */
	for (j = 0; j < max_len; ++j)
	  buf[j] = largeBuffer[j + whereInBuffer];

	*len = max_len;
	dataAvailable -= max_len;
	whereInBuffer += max_len;
      }

      return;
    }


    /* If we have no data left we need to get more */
    if (*(typePtr + (i * 16)) == 0xfa)
    {
      /* Bulk read */
      result = bulkRead (typePtr + (i * 16));
      dataAvailable =
	(*(typePtr + (i * 16) + 2) << 8) + *(typePtr + (i * 16) + 3);
      whereInBuffer = 0;
    }
    else
    {
      /* Control Transfer */
      result = controlTransfer (typePtr + (i * 16));
    }

    /* If something went wrong */
    if (result != 1)
    {
      fprintf (stderr, "******************\n");
      fprintf (stderr, "Something went wrong\n");
      fprintf (stderr, "Result not equal to 1\n");
      fprintf (stderr, "Error in 'Scanner Calibration'\n");

      if (dpiValue == 200)
	fprintf (stderr, "Urb %d and setup line %d\n", i + 936, i);
      else
	fprintf (stderr, "Urb %d and setup line %d\n", i + 1114, i);

      fprintf (stderr, "******************\n");
      exit (1);
    }
  }

  /* After scan, make sure to run remaining transfers */
  finalizeScanner ();

  /* Tell the program that we are ready to break out of the loop */
  tempVar = 1;
}


/*****************************************************************
 *  Main() - Runs through the program calling all of the SANE
 *           functions in the order that they are supposed to be
 *           called.
 *           If text is the first parameter on the command line
 *           the dpi will be set at 200.  Otherwise it will
 *           default to a color scan.
 *****************************************************************/
int main (int argc, char *argv[])
{

  if (argc > 1)
  {
    if (!strcmp (argv[1], "text"))
      dpiValue = 200;
  }

  fprintf (stderr, "DPI Value: %d\n", dpiValue);

  sane_init ();
  sane_getdevices ();

  struct usb_device dev;

  if (detectDevice (&dev))
  {

    sane_open ();
    sane_start ();

    if (dpiValue == 200)
      printf ("P2 1656 2342 255 ");
    else
      printf ("P3 826 1221 255 ");


    char *buffer = malloc (3000);
    int length = 0;

    while (tempVar != 1)
    {
      sane_read (buffer, 3000, &length);


      int i, j;
      for (i = 0; i < length; ++i)
      {
	/* Color scan */
	if (dpiValue == 100)
	{
	  printf ("%d ", (int) (buffer[i] & 0xff));
	}
	else			/* Black and white */
	{
	  for (j = 7; j > -1; --j)
	  {

	    if (((buffer[i] >> j) & 1) == 0)
	    {
	      printf ("0 ");
	    }
	    else
	    {
	      printf ("255 ");
	    }
	  }
	}
      }
    }

    free (buffer);
    sane_close ();
  }
  else
  {
    fprintf (stderr, "No Device Detected\n");
  }


  sane_exit ();
}

/****************************************************************
 *  Non-SANE functions  (Defined above)
 ****************************************************************/
int finalizeScanner ()
{
  int i;
  int result;

  for (i = 0; i < finalizeSize; i++)
  {
    /* Perform the transfers */
    result = controlTransfer (finalize[i]);

    /* If there was a problem */
    if (result < 0)
    {
      fprintf (stderr, "******************\n");
      fprintf (stderr, "Something went wrong\n");
      fprintf (stderr, "Result not equal to 1\n");
      fprintf (stderr, "Error in 'Finalize Scanner'\n");

      if (dpiValue == 200)
	fprintf (stderr, "Urb %d and setup line %d\n", i + 1071, i);
      else
	fprintf (stderr, "Urb %d and setup line %d\n", i + 1384, i);

      fprintf (stderr, "******************\n");

      exit (1);
    }
  }

  return 1;
}


int calibrate ()
{
  int ep = 2;
  int size = 0xc000;
  int result;

  char *buffer;
  buffer = largeBuffer;

  char temp;
  int incr = 0;
  int i;
  int j;

  /* Get the calibration info ready */
  for (i = 0; i < size; i += 64)
  {
    temp = (char) incr;

    for (j = 0; j < 64; ++j)
    {
      buffer[j + i] = temp;
    }

    incr++;

    if (incr > 0xff)
      incr = 0;

  }

  /* Send calibration data to the scanner */
  result = usb_bulk_write (deviceHandle, ep, buffer, size, 100);


  if (result > 0)
  {
    /* If the same size, we wrote all of the data */
    /* If not, we only wrote some of the data     */
    if (result == size)
      return 1;
    else
      return 2;
  }
  else
  {
    /* Nothing written */
    return 0;
  }
}


int calibrationWrite (int *data)
{
  /* This is a bulk write with specific data      */
  /* We complete the transfer by adding all zeros */
  /* Size needs to be 0x3000                      */

  int ep;
  int size;
  int j;
  int result;

  ep = data[1];
  size = (data[2] << 8) + data[3];

  char *buffer;
  buffer = largeBuffer;

  /* Zero out the buffer */
  for (j = 0; j < 0x3000; j++)
    largeBuffer[j] = 0;

  /* Transfer write data to buffer */
  for (j = 0; j < calibWriteSize; j++)
    largeBuffer[j] = calibWrite[j];

  /* Perform bulk write */
  result = usb_bulk_write (deviceHandle, ep, buffer, size, 100);

  if (result > 0)
  {
    /* If the same size, we wrote all of the data */
    /* If not, we only wrote some of the data     */
    if (result == size)
    {
      return 1;
    }
    else
    {
      return 2;
    }
  }
  else
  {
    /* Nothing written */
    return 0;
  }
}


int repeatedControlTransfer (int *data)
{
  int requestType;
  int request;
  int value;
  int index;
  int size;
  char checkCharacter;
  int result;

  /* determine data */
  requestType = data[1];
  request = data[2];
  value = (data[4] << 8) + data[3];
  index = (data[6] << 8) + data[5];
  size = (data[8] << 8) + data[7];

  char *buffer;
  buffer = largeBuffer;

  checkCharacter = (char) data[9];

  /* as soon as the scanner is ready, break the loop */
  while (((int) buffer[0] & 0xff) != ((int) checkCharacter & 0xff))
  {
    result = usb_control_msg (deviceHandle, requestType, request,
			      value, index, buffer, size, 300);

    if (result < 0)
    {
      /* Error somewhere */
      return 0;
    }
  }

  return 1;
}

int bulkRead (int *data)
{
  int ep;
  int size;
  int i;
  int result;

  ep = data[1];
  size = (data[2] << 8) + data[3];

  char *buffer;
  buffer = largeBuffer;

  for (i = 0; i < 10000; ++i);

  /* This timeout may need to be set higher than 2000 */
  result = usb_bulk_read (deviceHandle, ep, buffer, size, 2000);

  if (result > 0)
  {
    /* If the same size, we read all of the data */
    /* If not, we only read some of the data     */
    if (result == size)
      return 1;
    else
      return 2;
  }
  else
  {
    /* No data read */
    return 0;
  }
}


int writeBulk0s (int *data)
{
  int ep;
  int size;
  int i;
  int result;

  ep = data[1];
  size = (data[2] << 8) + data[1];

  char *buffer;
  buffer = largeBuffer;

  /* We can just overwrite largeBuffer because there is nothing */
  /*  of value there yet.                                       */
  for (i = 0; i <= size; ++i)
    largeBuffer[i] = 0;

  /* Perform the write */
  result = usb_bulk_write (deviceHandle, ep, buffer, size, 100);

  if (result > 0)
  {
    /* If the same size, we read all of the data */
    /* If not, we only read some of the data     */
    if (result == size)
      return 1;
    else
      return 2;
  }
  else
  {
    /* Nothing written */
    return 0;
  }
}

int controlTransfer (int *data)
{
  int requestType;
  int request;
  int value;
  int index;
  int size;
  int i;
  int result;

  requestType = data[0];
  request = data[1];
  value = (data[3] << 8) + data[2];
  index = (data[5] << 8) + data[4];
  size = (data[7] << 8) + data[6];

  /* this is where data will be read or written */
  char *buffer = largeBuffer;

  /* Loop here to transfer data that will be sent */
  for (i = 0; i < size; ++i)
  {
    /* Get the buffers ready for the control transfer */
    buffer[i] = (char) data[i + 8];
  }

  /* Perform the transfer */
  result = usb_control_msg (deviceHandle, requestType, request,
			    value, index, buffer, size, 300);

  if (result < 0)
  {
    /* Error during the control transfer */
    return 0;
  }

  /* Everything went as planned */
  return 1;
}


int detectDevice (struct usb_device *device)
{
  /* create variables */
  u_int16_t idVendor = 0x0461;
  u_int16_t idProduct = 0x0346;

  struct usb_bus *busses;
  struct usb_bus *bus;

  int deviceFound = 0;

  busses = usb_get_busses ();

  /* Match Colorado scanner to correct usb device. */
  for (bus = busses; bus; bus = bus->next)
  {
    struct usb_device *dev;

    for (dev = bus->devices; dev; dev = dev->next)
    {

      /* if Colorado 2400u is detected */
      if ((dev->descriptor.idVendor == idVendor) &&
	  (dev->descriptor.idProduct == idProduct))
      {
	/* Yes, it was detected */
	*device = *dev;

	return 1;
      }
    }
  }

  /* if not detected */
  if (!deviceFound)
  {
    return (0);
  }
}
