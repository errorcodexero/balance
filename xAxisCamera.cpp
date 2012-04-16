/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "xAxisCamera.h"

#include <string.h>
#include "NetworkCommunication/UsageReporting.h"
#include "Synchronized.h"
#include "Vision/PCVideoServer.h"
#include "WPIErrors.h"

#define CAMERA_FRAMES_PER_SECOND 6
#define CAMERA_COMPRESSION 60
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define CAMERA_ROTATION 0
#define CAMERA_BRIGHTNESS 50
#define CAMERA_COLOR 1
#define CAMERA_COLOR_LEVEL 50
#define CAMERA_AUTHENTICATION "RlJDOkZSQw==" // Username 'FRC', password 'FRC'.

/** Private NI function to decode JPEG */
IMAQ_FUNC int Priv_ReadJPEGString_C(Image* _image, const unsigned char* _string, UINT32 _stringLength);

// Max packet without jumbo frames is 1500... add 36 because??
#define kMaxPacketSize 1536
#define kImageBufferAllocationIncrement 1000

xAxisCamera *xAxisCamera::_instance = NULL;

/**
 * xAxisCamera constructor
 */
xAxisCamera::xAxisCamera(const char *ipAddress)
	: AxisCameraParams(ipAddress)
	, m_cameraSocket(ERROR)
	, m_protectedImageBuffer(NULL)
	, m_protectedImageBufferLength(0)
	, m_protectedImageSize(0)
	, m_protectedImageSem(NULL)
	, m_freshImage(false)
	, m_imageStreamTask("cameraTask", (FUNCPTR)s_ImageStreamTaskFunction)
	, m_videoServer(NULL)
{
	m_protectedImageSem = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE | SEM_DELETE_SAFE);

#if JAVA_CAMERA_LIB != 1
	nUsageReporting::report(nUsageReporting::kResourceType_AxisCamera, ipAddress == NULL ? 1 : 2);
#endif

	if (!StatusIsFatal())
		m_imageStreamTask.Start((int)this); 
}

/**
 * Destructor
 */
xAxisCamera::~xAxisCamera()
{
	delete m_videoServer;
	m_videoServer = NULL;

	m_imageStreamTask.Stop();
	close(m_cameraSocket);

	SemSet_t::iterator it = m_newImageSemSet.begin();
	SemSet_t::iterator end = m_newImageSemSet.end();
	for (;it != end; it++)
	{
		semDelete(*it);
	}
	m_newImageSemSet.clear();

	semDelete(m_protectedImageSem);
}

/**
 * Get a pointer to the xAxisCamera object, if the object does not exist, create it
 * To use the camera on port 2 of a cRIO-FRC, pass "192.168.0.90" to the first GetInstance call.
 * @return reference to xAxisCamera object
 */
xAxisCamera &xAxisCamera::GetInstance(const char *cameraIP)
{
	if (NULL == _instance)
	{
		_instance = new xAxisCamera(cameraIP);

		_instance->m_videoServer = new PCVideoServer();
	}

	return *_instance;
}

/**
 * Called by Java to delete the camera... how thoughtful
 */
void xAxisCamera::DeleteInstance()
{
	delete _instance;
	_instance = NULL;
}

/**
 * Return true if the latest image from the camera has not been retrieved by calling GetImage() yet.
 * @return true if the image has not been retrieved yet.
 */
bool xAxisCamera::IsFreshImage()
{
	return m_freshImage;
}

/**
 * Get the semaphore to be used to synchronize image access with camera acquisition
 *
 * Call semTake on the returned semaphore to block until a new image is acquired.
 *
 * The semaphore is owned by the xAxisCamera class and will be deleted when the class is destroyed.
 * @return A semaphore to notify when new image is received
 */
SEM_ID xAxisCamera::GetNewImageSem()
{
	SEM_ID sem = semBCreate (SEM_Q_PRIORITY, SEM_EMPTY);
	m_newImageSemSet.insert(sem);
	return sem;
}

/**
 * Get an image from the camera and store it in the provided image.
 * @param image The imaq image to store the result in. This must be an HSL or RGB image
 * This function is called by Java.
 * @return 1 upon success, zero on a failure
 */
int xAxisCamera::GetImage(Image* imaqImage)
{
	if (m_protectedImageBuffer == NULL)
		return 0;
	Synchronized sync(m_protectedImageSem);
	Priv_ReadJPEGString_C(imaqImage,
		(unsigned char*)m_protectedImageBuffer, m_protectedImageSize);
	m_freshImage = false;
	return 1;
}

#if JAVA_CAMERA_LIB != 1
/**
 * Get an image from the camera and store it in the provided image.
 * @param image The image to store the result in. This must be an HSL or RGB image
 * @return 1 upon success, zero on a failure
 */
int xAxisCamera::GetImage(ColorImage* image)
{
	return GetImage(image->GetImaqImage());
}

/**
 * Instantiate a new image object and fill it with the latest image from the camera.
 *
 * The returned pointer is owned by the caller and is their responsibility to delete.
 * @return a pointer to an HSLImage object
 */
HSLImage* xAxisCamera::GetImage()
{
	HSLImage *image = new HSLImage();
	GetImage(image);
	return image;
}
#endif

/**
 * Copy an image into an existing buffer.
 * This copies an image into an existing buffer rather than creating a new image
 * in memory. That way a new image is only allocated when the image being copied is
 * larger than the destination.
 * This method is called by the PCVideoServer class.
 * @param imageData The destination image.
 * @param numBytes The size of the destination image.
 * @return 0 if failed (no source image or no memory), 1 if success.
 */
int xAxisCamera::CopyJPEG(char **destImage, int &destImageSize, int &destImageBufferSize)
{
	Synchronized sync(m_protectedImageSem);
	if (destImage == NULL)
		wpi_setWPIErrorWithContext(NullParameter, "destImage must not be NULL");

	if (m_protectedImageBuffer == NULL || m_protectedImageSize <= 0)
		return 0; // if no source image

	if (destImageBufferSize < m_protectedImageSize) // if current destination buffer too small
	{
		if (*destImage != NULL) delete [] *destImage;
		destImageBufferSize = m_protectedImageSize + kImageBufferAllocationIncrement;
		*destImage = new char[destImageBufferSize];
		if (*destImage == NULL) return 0;
	}
	// copy this image into destination buffer
	if (*destImage == NULL)
	{
		wpi_setWPIErrorWithContext(NullParameter, "*destImage must not be NULL");
	}
	// TODO: Is this copy realy necessary... perhaps we can simply transmit while holding the protected buffer
	memcpy(*destImage, m_protectedImageBuffer, m_protectedImageSize);
	destImageSize = m_protectedImageSize;
	return 1;
}

/**
 * Static interface that will cause an instantiation if necessary.
 * This static stub is directly spawned as a task to read images from the camera.
 */
int xAxisCamera::s_ImageStreamTaskFunction(xAxisCamera *thisPtr)
{
	return thisPtr->ImageStreamTaskFunction();
}

/**
 * Task spawned by xAxisCamera constructor to receive images from cam
 * If setNewImageSem has been called, this function does a semGive on each new image
 * Images can be accessed by calling getImage()
 */
int xAxisCamera::ImageStreamTaskFunction()
{
	// Loop on trying to setup the camera connection. This happens in a background
	// thread so it shouldn't effect the operation of user programs.
	while (1)
	{

#define	STR(x) QUOTE(X)
#define	QUOTE(X) #X

		const char *requestString = "GET /axis-cgi/mjpg/video.cgi"
					    "?des_fps=" STR(CAMERA_FRAMES_PER_SECOND)
					    "&compression=" STR(CAMERA_COMPRESSION)
					    "&resolution=" STR(CAMERA_WIDTH) "x" STR(CAMERA_HEIGHT)
					    "&rotation=" STR(CAMERA_ROTATION)
					    "&brightness=" STR(CAMERA_BRIGHTNESS)
					    "&color=" STR(CAMERA_COLOR)
					    "&colorlevel=" STR(CAMERA_COLOR_LEVEL)
					    " HTTP/1.1\r\n"
					    "User-Agent: HTTPStreamClient\r\n"
					    "Accept: */*\r\n"
					    "Connection: Close\r\n"
					    "Authorization: Basic " STR(CAMERA_AUTHENTICATION) ";\r\n\r\n";

		semTake(m_socketPossessionSem, WAIT_FOREVER);
		m_cameraSocket = CreateCameraSocket(requestString);
		if (m_cameraSocket == ERROR)
		{
			// Don't hammer the camera if it isn't ready.
			semGive(m_socketPossessionSem);
			taskDelay(1000);
		}
		else
		{
			ReadImagesFromCamera();
		}
	}
}

/**
 * This function actually reads the images from the camera.
 */
int xAxisCamera::ReadImagesFromCamera()
{
	char *imgBuffer = NULL;
	int imgBufferLength = 0;
	//Infinite loop, task deletion handled by taskDeleteHook
	// Socket cleanup handled by destructor

	// TODO: these recv calls must be non-blocking. Otherwise if the camera
	// fails during a read, the code hangs and never retries when the camera comes
	// back up.

	int counter = 2;
	while (1)
	{
		char initialReadBuffer[kMaxPacketSize] = "";
		char intermediateBuffer[1];
		char *trailingPtr = initialReadBuffer;
		int trailingCounter = 0;
		while (counter)
		{
			// TODO: fix me... this cannot be the most efficient way to approach this, reading one byte at a time.
			if(recv(m_cameraSocket, intermediateBuffer, 1, 0) == ERROR)
			{
				wpi_setErrnoErrorWithContext("Failed to read image header");
				close (m_cameraSocket);
				return ERROR;
			}
			strncat(initialReadBuffer, intermediateBuffer, 1);
			// trailingCounter ensures that we start looking for the 4 byte string after
			// there is at least 4 bytes total. Kind of obscure.
			// look for 2 blank lines (\r\n)
			if (NULL != strstr(trailingPtr, "\r\n\r\n"))
			{
				--counter;
			}
			if (++trailingCounter >= 4)
			{
				trailingPtr++;
			}
		}
		counter = 1;
		char *contentLength = strstr(initialReadBuffer, "Content-Length: ");
		if (contentLength == NULL)
		{
			wpi_setWPIErrorWithContext(IncompatibleMode, "No content-length token found in packet");
			close(m_cameraSocket);
			return ERROR;
		}
		contentLength = contentLength + 16; // skip past "content length"
		int readLength = atol(contentLength); // get the image byte count

		// Make sure buffer is large enough
		if (imgBufferLength < readLength)
		{
			if (imgBuffer) delete[] imgBuffer;
			imgBufferLength = readLength + kImageBufferAllocationIncrement;
			imgBuffer = new char[imgBufferLength];
			if (imgBuffer == NULL)
			{
				imgBufferLength = 0;
				continue;
			}
		}

		// Read the image data for "Content-Length" bytes
		int bytesRead = 0;
		int remaining = readLength;
		while(bytesRead < readLength)
		{
			int bytesThisRecv = recv(m_cameraSocket, &imgBuffer[bytesRead], remaining, 0);
			bytesRead += bytesThisRecv;
			remaining -= bytesThisRecv;
		}
		// Update image
		UpdatePublicImageFromCamera(imgBuffer, readLength);
		if (semTake(m_paramChangedSem, NO_WAIT) == OK)
		{
			// params need to be updated: close the video stream; release the camera.
			close(m_cameraSocket);
			semGive(m_socketPossessionSem);
			return 0;
		}
	}
}

/**
 * Copy the image from private buffer to shared buffer.
 * @param imgBuffer The buffer containing the image
 * @param bufLength The length of the image
 */
void xAxisCamera::UpdatePublicImageFromCamera(char *imgBuffer, int imgSize)
{
	{
		Synchronized sync(m_protectedImageSem);

		// Adjust the buffer size if current destination buffer is too small.
		if (m_protectedImageBufferLength < imgSize)
		{
			if (m_protectedImageBuffer != NULL) delete [] m_protectedImageBuffer;
			m_protectedImageBufferLength = imgSize + kImageBufferAllocationIncrement;
			m_protectedImageBuffer = new char[m_protectedImageBufferLength];
			if (m_protectedImageBuffer == NULL)
			{
				m_protectedImageBufferLength = 0;
				return;
			}
		}

		memcpy(m_protectedImageBuffer, imgBuffer, imgSize);
		m_protectedImageSize = imgSize;
	}

	m_freshImage = true;
	// Notify everyone who is interested.
	SemSet_t::iterator it = m_newImageSemSet.begin();
	SemSet_t::iterator end = m_newImageSemSet.end();
	for (;it != end; it++)
	{
		semGive(*it);
	}
}

/**
 * Implement the pure virtual interface so that when parameter changes require a restart, the image task can be bounced.
 */
void xAxisCamera::RestartCameraTask()
{
	m_imageStreamTask.Stop();
	m_imageStreamTask.Start((int)this);
}

#if JAVA_CAMERA_LIB == 1

// C bindings used by Java
// These need to stay as is or Java has to change

void AxisCameraStart(const char *IPAddress)
{
#ifdef SVN_REV
	if (strlen(SVN_REV))
	{
		printf("JavaCameraLib was compiled from SVN revision %s\n", SVN_REV);
	}
	else
	{
		printf("JavaCameraLib was compiled from a location that is not source controlled.\n");
	}
#else
	printf("JavaCameraLib was compiled without -D'SVN_REV=nnnn'\n");
#endif
	xAxisCamera::GetInstance(IPAddress);
}

int AxisCameraGetImage (Image* image)
{
	return xAxisCamera::GetInstance().GetImage(image);
}

void AxisCameraWriteBrightness(int brightness)
{
	xAxisCamera::GetInstance().WriteBrightness(brightness);
}

int AxisCameraGetBrightness()
{
	return xAxisCamera::GetInstance().GetBrightness();
}

void AxisCameraWriteWhiteBalance(AxisCameraParams::WhiteBalance_t whiteBalance)
{
	xAxisCamera::GetInstance().WriteWhiteBalance(whiteBalance);
}

AxisCameraParams::WhiteBalance_t AxisCameraGetWhiteBalance()
{
	return xAxisCamera::GetInstance().GetWhiteBalance();
}

void AxisCameraWriteColorLevel(int colorLevel)
{
	xAxisCamera::GetInstance().WriteColorLevel(colorLevel);
}

int AxisCameraGetColorLevel()
{
	return xAxisCamera::GetInstance().GetColorLevel();
}

void AxisCameraWriteExposureControl(AxisCameraParams::Exposure_t exposure)
{
	xAxisCamera::GetInstance().WriteExposureControl(exposure);
}

AxisCameraParams::Exposure_t AxisCameraGetExposureControl()
{
	return xAxisCamera::GetInstance().GetExposureControl();
}

void AxisCameraWriteExposurePriority(int exposure)
{
	xAxisCamera::GetInstance().WriteExposurePriority(exposure);
}

int AxisCameraGetExposurePriority()
{
	return xAxisCamera::GetInstance().GetExposurePriority();
}

void AxisCameraWriteMaxFPS(int maxFPS)
{
	xAxisCamera::GetInstance().WriteMaxFPS(maxFPS);
}

int AxisCameraGetMaxFPS()
{
	return xAxisCamera::GetInstance().GetMaxFPS();
}

void AxisCameraWriteResolution(AxisCameraParams::Resolution_t resolution)
{
	xAxisCamera::GetInstance().WriteResolution(resolution);
}

AxisCameraParams::Resolution_t AxisCameraGetResolution()
{
	return xAxisCamera::GetInstance().GetResolution();
}

void AxisCameraWriteCompression(int compression)
{
	xAxisCamera::GetInstance().WriteCompression(compression);
}

int AxisCameraGetCompression()
{
	return xAxisCamera::GetInstance().GetCompression();
}

void AxisCameraWriteRotation(AxisCameraParams::Rotation_t rotation)
{
	xAxisCamera::GetInstance().WriteRotation(rotation);
}

AxisCameraParams::Rotation_t AxisCameraGetRotation()
{
	return xAxisCamera::GetInstance().GetRotation();
}

void AxisCameraDeleteInstance()
{
	xAxisCamera::DeleteInstance();
}

int AxisCameraFreshImage()
{
	return xAxisCamera::GetInstance().IsFreshImage();
}

#endif // JAVA_CAMERA_LIB == 1

