/*******************************************************************************
 *  Primascan.c
 *
 *  Purpose: To provide a driver that can be used on a linux operating system
 *           and will be able to scan from the Primax Colorado 2400u scanner.
 *           This driver was designed to work with the SANE Api.
 *
 *  Author:  Richard Murri
 *  Date:    Nov 16, 2005
 *     
   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation; either version 2 of the
   License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston,
   MA 02111-1307, USA.

   As a special exception, the authors of SANE give permission for
   additional uses of the libraries contained in this release of SANE.

   The exception is that, if you link a SANE library with other files
   to produce an executable, this does not by itself cause the
   resulting executable to be covered by the GNU General Public
   License.  Your use of that executable is in no way restricted on
   account of linking the SANE library code into it.

   This exception does not, however, invalidate any other reasons why
   the executable file might be covered by the GNU General Public
   License.

   If you submit changes to SANE to the maintainers to be included in
   a subsequent release, you agree by submitting the changes that
   those changes may be distributed with this exception intact.

   If you write modifications of your own for SANE, it is your choice
   whether to permit this exception to apply to your modifications.
   If you do not wish that, delete this exception notice.  

 ******************************************************************************/
#include "sane/config.h"
#include "sane/sane.h"
#include "sane/sanei.h"

#include <stdio.h>
#include <string.h>
#include <usb.h>

#define BACKEND_NAME primascan
#define BUILD 1

#include <sane/saneopts.h>
#include "sane/sanei_config.h"
#include "sane/sanei_backend.h"



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

/* This holds information about what devices are opened
   Used in sane_get_devices()                           */
static SANE_Device **deviceArray = 0;
static SANE_Device *colorado;

/* Option descriptors */
static SANE_Option_Descriptor number0;
static SANE_Option_Descriptor number1;
static SANE_Int word_list[3];



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
int detectDevice (struct usb_device *device);
int controlTransfer (int *data);
int writeBulk0s (int *data);
int bulkRead (int *data);
int repeatedControlTransfer (int *data);
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
SANE_Status sane_init (SANE_Int * version_code, SANE_Auth_Callback authorize)
{
  /* prevent compiler from giving warnings */
  authorize = authorize;

  /* These functions need called before we can use the classes */
  usb_init ();
  usb_find_busses ();
  usb_find_devices ();

  /* Set up the version */
  if (version_code != NULL)
  {
    *version_code = SANE_VERSION_CODE (V_MAJOR, V_MINOR, BUILD);
  }

  isDeviceOpen = 0;

  return SANE_STATUS_GOOD;
}

void sane_exit (void)
{
  SANE_Handle h = NULL;
  sane_close (h);

  /* Free the memory we've already used */
  if (colorado != NULL)
    free (colorado);

  if (deviceArray != NULL)
    free (deviceArray);

}

SANE_Status
sane_get_devices (const SANE_Device *** device_list, SANE_Bool local_only)
{
  /* prevent compiler from complaining about unused parameters */
  local_only = local_only;

  int deviceFound = 0;
  struct usb_device dev;

  /* Find attached Colorodo scanner information */
  deviceFound = detectDevice (&dev);

  if (deviceFound)
  {
    /* Free the memory we might have already used */
    if (colorado != NULL)
      free (colorado);

    if (deviceArray != NULL)
      free (deviceArray);

    /* Set up the colorado scanner */
    colorado = malloc (sizeof (SANE_Device));

    colorado->name = "Primascan";
    colorado->vendor = "Primax";
    colorado->model = "Colorado 2400u";
    colorado->type = "flatbed scanner";

    /* Set up the array of deviced to pass back */
    deviceArray = malloc (2 * sizeof (SANE_Device));

    if (deviceArray == NULL)
      return SANE_STATUS_NO_MEM;

    deviceArray[0] = colorado;
    deviceArray[1] = NULL;

    /* Let SANE know that the scanner is attached */
    *device_list = (const SANE_Device **) deviceArray;

    return SANE_STATUS_GOOD;
  }

  /* If the device is not found */
  return (SANE_STATUS_GOOD);
}

SANE_Status sane_open (SANE_String_Const devicename, SANE_Handle * handle)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;
  devicename = devicename;

  if (!isDeviceOpen)
  {
    struct usb_device dev;
    if (detectDevice (&dev))
    {
      /* Open device */
      deviceHandle = usb_open (&dev);

      int status1;
      int status2;
      int status3;

      /* Get configuration ready */
      status1 = usb_set_configuration (deviceHandle, 1);
      status2 = usb_claim_interface (deviceHandle, 0);
      status3 = usb_set_altinterface (deviceHandle, 0);

      /* If any of the configuration fails */
      if ((deviceHandle == NULL) || (status1 < 0) ||
	  (status2 < 0) || (status3 < 0))
      {
	/* ERROR, DEVICE NOT OPEN */
	return SANE_STATUS_IO_ERROR;
      }

      isDeviceOpen = 1;

      return SANE_STATUS_GOOD;
    }
    else
    {
      /* ERROR, DEVICE NOT AVAILABLE */
      return SANE_STATUS_INVAL;
    }

  }

  /* if it's already opened */
  return SANE_STATUS_GOOD;
}

void sane_close (SANE_Handle handle)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  /* Close any open device  */
  if (isDeviceOpen)
  {
    usb_release_interface(deviceHandle, 0);
    usb_close(deviceHandle);
    isDeviceOpen = 0;
  }
}

const SANE_Option_Descriptor *sane_get_option_descriptor (SANE_Handle handle,
							  SANE_Int option)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  number0.name = "";
  number0.title = SANE_TITLE_NUM_OPTIONS;
  number0.desc = SANE_DESC_NUM_OPTIONS;
  number0.type = SANE_TYPE_INT;
  number0.unit = SANE_UNIT_NONE;
  number0.size = sizeof (SANE_Word);
  number0.cap = SANE_CAP_SOFT_DETECT;
  number0.constraint_type = SANE_CONSTRAINT_NONE;
  number0.constraint.range = 0;

  number1.name = "dpi";
  number1.title = "dpi setting";
  number1.desc =  "This controls the dpi setting of the scanner. 100 if a color scan and 200 is a black and white scan";
  number1.type = SANE_TYPE_INT;
  number1.unit = SANE_UNIT_NONE;
  number1.size = sizeof (SANE_Word);
  number1.cap = SANE_CAP_SOFT_SELECT;
  number1.constraint_type = SANE_CONSTRAINT_WORD_LIST;
  number1.constraint.range = 0;

  word_list[0] = 2;
  word_list[1] = 100;
  word_list[2] = 200;

  number1.constraint.word_list = word_list;

  if (option == 0)
    return &number0;
  else
    return &number1;
}

SANE_Status
sane_control_option (SANE_Handle handle, SANE_Int option,
		     SANE_Action action, void *val, SANE_Int * info)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;
  info = info;

  /* Option dealing with dpi */
  if (option == 1)	
  {
    /* Get current value */
    if (action == 0)
    {
      val = (void *) dpiValue;
    }
    /* Set the value */
    else if (action == 1)
    {
      int temp = *(int *) val;
      dpiValue = temp;
    }
    else
    {
      dpiValue = 100;
    }
  }
  /* Option dealing with number of options */
  else if (option == 0)
  {
    *(SANE_Word *) val = 2;
  }

  else
    return SANE_STATUS_UNSUPPORTED;

  return SANE_STATUS_GOOD;
}

SANE_Status sane_get_parameters (SANE_Handle handle, SANE_Parameters * params)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  /* Black and white scan */
  if (dpiValue == 200)
  {	
    params->format = SANE_FRAME_GRAY;
    params->last_frame = 1;
    params->lines = 2342;
    params->depth = 1;
    params->pixels_per_line = 1656;
    params->bytes_per_line = 207;
  }
  /* Color scan */
  else
  {			 
    params->format = SANE_FRAME_RGB;
    params->last_frame = 1;
    params->lines = 1221;
    params->depth = 8;	
    params->pixels_per_line = 826;
    params->bytes_per_line = 2478;
  }

  return SANE_STATUS_GOOD;
}

SANE_Status sane_start (SANE_Handle handle)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  int i;
  int result;

  /* Initialize scanner */
  for (i = 0; i < scannerSetupSize; i++)
  {
    result = controlTransfer (scannerSetup[i]);

    if (result != 1)
    {
      return SANE_STATUS_IO_ERROR;
    }
  }

  /* Scanner setup */
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

  /* print out values */
  for (i = 0; i < typeSize; i++)
  {
    /* We need to test for the different possibilities */
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
      /* Something went wrong, but it won't effect the scan any */
    }
  }

  /* Scanner Calibration */
  for (i = 0; i < calibrationSize; i++)
  {
    if (calibration[i][0] == 0xfc)
    {
      /* Special calibration */
      result = calibrationWrite (calibration[i]);
    }
    else if (calibration[i][0] == 0xfd)
    {
      /* calibrate */
      result = calibrate ();
    }
    else
    {
      /* Normal control transfer */
      result = controlTransfer (calibration[i]);
    }

    if (result != 1)
    {
      return SANE_STATUS_IO_ERROR;
    }
  }

  return SANE_STATUS_GOOD;
}

SANE_Status
sane_read (SANE_Handle handle, SANE_Byte * buf, SANE_Int max_len,
	   SANE_Int * len)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  /*  We want this to remain between function calls
   *  Essential we need to do this read across several function calls
   *     as if it was only one call 
   */

  static int dataAvailable = 0;	
  static int whereInBuffer = 0;	
  static int i = 0;

  int *typePtr;
  int typeSize;
  int result;

  /* Setup is different for black or for color */
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

  /* perform scan one step at a time */

  /* Don't reinitialize i, we need it to be static */
  for (; i < typeSize; i++)
  {

    /* 
     *  If we didn't finish giving all of our data to the function
     *  last time because the buffer was too small, do it now.
     */

    if (dataAvailable > 0)
    {
      if (dataAvailable < max_len)
      {
	/* copy data to buffer */
	int j;

	if (dpiValue == 200)
	{
	  /* In text mode we need to flip the bits so it turns out right */
	  for (j = 0; j < dataAvailable; ++j)
	  {
	    largeBuffer[j + whereInBuffer] = ~largeBuffer[j + whereInBuffer];
	  }
	}

	for (j = 0; j < dataAvailable; ++j)
	{
	  buf[j] = largeBuffer[j + whereInBuffer];
	}

	*len = dataAvailable;
	dataAvailable = 0;
	whereInBuffer = 0;
      }
      else
      {
	int j;
	for (j = 0; j < max_len; ++j)
	{
	  buf[j] = largeBuffer[j + whereInBuffer];
	}

	*len = max_len;
	dataAvailable -= max_len;
	whereInBuffer += max_len;
      }

      return SANE_STATUS_GOOD;
    }


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

    if (result != 1)
    {
      return SANE_STATUS_IO_ERROR;
    }
  }

  finalizeScanner ();

  return SANE_STATUS_EOF;
}

void sane_cancel (SANE_Handle handle)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  finalizeScanner ();
  usb_reset (deviceHandle);
  isDeviceOpen = 0;
}

SANE_Status sane_set_io_mode (SANE_Handle handle, SANE_Bool non_blocking)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;

  if (non_blocking == SANE_TRUE)
    return SANE_STATUS_UNSUPPORTED;

  return SANE_STATUS_GOOD;
}

SANE_Status sane_get_select_fd (SANE_Handle handle, SANE_Int * fd)
{
  /* prevent compiler from complaining about unused parameters */
  handle = handle;
  fd = fd;

  return SANE_STATUS_UNSUPPORTED;
}


/*******************************************************************
 *  Non-SANE functions (Defined above)
 ******************************************************************/

int detectDevice (struct usb_device *device)
{
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

  result = usb_bulk_write (deviceHandle, ep, buffer, size, 100);

  if (result > 0)
  {
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
    /* Complete write failure */
    return 0;
  }
}


int calibrationWrite (int *data)
{
  /* 
   * bulk write with specific data
   * Remember it needs completion.
   * Size needs to be 0x3000
   */

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

  result = usb_bulk_write (deviceHandle, ep, buffer, size, 100);

  if (result > 0)
  {
    /* At least we've written something */

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
    /* Complete write failure */
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

  /* determine data from file */
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

  result = usb_bulk_read (deviceHandle, ep, buffer, size, 3000);

  if (result > 0)
  {
    /* At least we've read something */

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
    /* Complete read failure */
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

  /*
   *  We can just overwrite largeBuffer because there is nothing
   *  of value there yet.
   */
  for (i = 0; i <= size; ++i)
    largeBuffer[i] = 0;

  result = usb_bulk_write (deviceHandle, ep, buffer, size, 100);

  if (result > 0)
  {
    /* At least we've written something */

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
    /* Complete write failure */
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

  /* determine data from file */
  requestType = data[0];
  request = data[1];
  value = (data[3] << 8) + data[2];
  index = (data[5] << 8) + data[4];
  size = (data[7] << 8) + data[6];

  /* this is where data will be read or written */
  char *buffer = largeBuffer;

  /* Loop here to read in data. */

  for (i = 0; i < size; ++i)
  {
    /* Get the buffers ready for the control transfer */
    buffer[i] = (char) data[i + 8];
  }

  /* Display what the result is */
  result = usb_control_msg (deviceHandle, requestType, request,
			    value, index, buffer, size, 300);

  if (result < 0)
  {
    /* Error */
    return 0;
  }

  /* All is well */
  return 1;
}


int finalizeScanner ()
{
  int i;
  int result;

  for (i = 0; i < finalizeSize; i++)
  {
    result = controlTransfer (finalize[i]);

    if (result < 0)
    {
      /* Error */
      exit (1);
    }
  }

  return 1;
}
