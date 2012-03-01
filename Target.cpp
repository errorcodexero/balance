// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include "Target.h"
#include "Version.h"
#include <math.h>

static Version v( __FILE__ " " __DATE__ " " __TIME__ );

Target::Target()
{
    memset(falseColor, 0, sizeof(falseColor));
    for (int i = 1; i < 255; i++) {
	switch (i % 6) {
	case 0:	// white
	    falseColor[i].R = falseColor[i].G = falseColor[i].B = 255;
	    break;
	case 1: // magenta
	    falseColor[i].B = falseColor[i].R = 255;
	    break;
	case 2: // red
	    falseColor[i].R = 255;
	    break;
	case 3: // yellow
	    falseColor[i].R = falseColor[i].G = 255;
	    break;
	case 4: // green
	    falseColor[i].G = 255;
	    break;
	case 5: // cyan
	    falseColor[i].G = falseColor[i].B = 255;
	    break;
	}
    }

    filterCriteria[0].parameter  = IMAQ_MT_PARTICLE_AND_HOLES_AREA;
    filterCriteria[0].lower      = 0;
    filterCriteria[0].upper      = 400;
    filterCriteria[0].calibrated = FALSE;
    filterCriteria[0].exclude    = TRUE;

    filterOptions.rejectMatches = FALSE;
    filterOptions.rejectBorder  = FALSE;
    filterOptions.fillHoles     = TRUE;
    filterOptions.connectivity8 = TRUE;
}


Target::~Target()
{
    ;
}


bool Target::GetImage()
{
    long then = (long) GetFPGATime();

    AxisCamera& axisCamera = AxisCamera::GetInstance();
    if (!axisCamera.IsFreshImage()) {
	return false;
    }
    if (!axisCamera.GetImage(&cameraImage)) {
	printf("%s: image acquisition FAILED\n", __FUNCTION__);
	return false;
    }

    long now = (long) GetFPGATime();
    printf("%s: image acquisition took %ld microseconds\n", __FUNCTION__, (now - then));

    return true;
}

void Target::SaveImages()
{
    long then = (long) GetFPGATime();

    if (!imaqWriteFile(cameraImage.GetImaqImage(), "/tmp/vision00-camera.bmp", NULL)) {
	printf("%s: imaqWriteFile(\"/tmp/vision00-camera.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }
    if (!imaqWriteFile(monoImage.GetImaqImage(), "/tmp/vision01-monoImage.bmp", NULL)) {
	printf("%s: imaqWriteFile(\"/tmp/vision01-luminance.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }
#if 0
    if (!imaqWriteFile(equalized.GetImaqImage(), "/tmp/vision02-equalized.bmp", NULL)) {
	printf("%s: imaqWriteFile(\"/tmp/vision02-equalized.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }
#endif
    if (!imaqWriteFile(filtered.GetImaqImage(), "/tmp/vision03-filtered.bmp", falseColor)) {
	printf("%s: imaqWriteFile(\"/tmp/vision03-filtered.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }

    long now = (long) GetFPGATime();
    printf("%s: image save took %ld microseconds\n", __FUNCTION__, (now - then));
}

int Target::FindParticles()
{
    long then = (long) GetFPGATime();
    MonoImage *pImage = NULL;

    // extract the blue plane
    if (!imaqExtractColorPlanes(cameraImage.GetImaqImage(), IMAQ_RGB,
    		NULL, NULL, monoImage.GetImaqImage()))
    {
	printf("%s: imaqExtractColorPlanes FAILED\n", __FUNCTION__);
	return 0;
    }
    pImage = &monoImage;

#if 0
    // apply auto-median to simplify particles
    if (!imaqGrayMorphology(equalized.GetImaqImage(), monoImage.GetImaqImage(),
    		IMAQ_AUTOM, NULL))
    {
	printf("%s: imaqGrayMorphology FAILED\n", __FUNCTION__);
	return 0;
    }
    pImage = &equalized;
#endif

    // select interesting particles
    int particleCount = 0;
    if (!imaqParticleFilter4(filtered.GetImaqImage(), pImage->GetImaqImage(),
    		filterCriteria, 1, &filterOptions, NULL, &particleCount))
    {
	printf("%s: imaqParticleFilter FAILED\n", __FUNCTION__);
	return 0;
    }

    // select the four largest particles (insertion sort)
    // for now, keep track of only the particle number (index) and size
    memset((void *)particles, 0, sizeof particles);
    for (int i = 0; i < particleCount; i++) {
	double size;
	if (!imaqMeasureParticle(filtered.GetImaqImage(), i, FALSE,
	    IMAQ_MT_PARTICLE_AND_HOLES_AREA, &size))
	{
	    printf("%s: imaqMeasureParticle %d FAILED\n", __FUNCTION__, i);
	    break;
	}
	for (int j = 0; j < 4; j++) {
	    if (size > particles[j].size) {
		for (int k = 3; k > j; k--) {
		    particles[k].index = particles[k-1].index;
		    particles[k].size = particles[k-1].size;
		}
		particles[j].index = i;
		particles[j].size = size;
		break;
	    }
	}
    }

    // fill in the rest of the measured data
    for (num_particles = 0;
    	 num_particles < 4 && particles[num_particles].size > 0;
	 num_particles++)
    {
	Particle* p = &particles[num_particles];
	imaqMeasureParticle(filtered.GetImaqImage(), num_particles, FALSE,
			    IMAQ_MT_CENTER_OF_MASS_X, &(p->xCenter));
	imaqMeasureParticle(filtered.GetImaqImage(), num_particles, FALSE,
			    IMAQ_MT_CENTER_OF_MASS_Y, &(p->yCenter));
	imaqMeasureParticle(filtered.GetImaqImage(), num_particles, FALSE,
			    IMAQ_MT_BOUNDING_RECT_LEFT, &(p->leftBound));
	imaqMeasureParticle(filtered.GetImaqImage(), num_particles, FALSE,
			    IMAQ_MT_BOUNDING_RECT_RIGHT, &(p->rightBound));
	imaqMeasureParticle(filtered.GetImaqImage(), num_particles, FALSE,
			    IMAQ_MT_BOUNDING_RECT_TOP, &(p->topBound)); 
	imaqMeasureParticle(filtered.GetImaqImage(), num_particles, FALSE,
			    IMAQ_MT_BOUNDING_RECT_BOTTOM, &(p->bottomBound));
	p->height = p->bottomBound - p->topBound;
	p->width = p->rightBound - p->leftBound;
    }

    long now = (long) GetFPGATime();
    printf("%s: particle detection took %ld microseconds\n", __FUNCTION__, (now - then));
    printf("%s: found %d particles\n", __FUNCTION__, particleCount);
    printf("%s: returning %d particles\n", __FUNCTION__, num_particles);
    for (int i = 0; i < num_particles; i++) {
	Particle *p = &particles[i];
	printf("  particle %d size %g x %g y %g\n",
		p->index, p->size, p->xCenter, p->yCenter);
    }

    return num_particles;
}

bool Target::AnalyzeParticles()
{
    if (num_particles < 4) {
	printf("ERROR: num_particles < 4, no analysis possible\n");
	return false;
    }

    // image quality tests:
    //
    // 1) There should be some space between the outer edge of each particle and the edge
    // of the image.  If not, one of the particles is partially outside the field of view.
    //
    // 2) The four largest particles should be approximately the same size.  If not,
    // one or more of the particles is partially hidden or we didn't identify the particles
    // correctly.
    //
    // 3) The aspect ratio (height to width ratio) of each particle should be approximately
    // that of the target rectangles.
    //
    // 4) The image center position calculated from the inside edges of the particles,
    // the center of mass of the particles, and the outside edges of the particles should
    // be approximately the same.

    for (int i = 1; i < num_particles; i++) {
	Particle *p = &particles[i];
	if (p->size < particles[0].size * 0.4) {
	    printf("WARNING: particle %d clipped\n", i);
	    break;
	}
    }

    // Sort the particles by position, based on their inside edges.
    pTop = pBottom = pLeft = pRight = &particles[0];

    for (int i = 1; i < num_particles; i++) {
	Particle *p = &particles[i];
	if (p->bottomBound > pTop->bottomBound) {
	    pTop = p;
	}
	if (p->topBound < pBottom->topBound) {
	    pBottom = p;
	}
	if (p->rightBound < pLeft->rightBound) {
	    pLeft = p;
	}
	if (p->leftBound > pRight->leftBound) {
	    pRight = p;
	}
    }

    printf("top %d left %d bottom %d right %d\n",
	   pTop->index, pLeft->index, pBottom->index, pRight->index);

    // Find the center of the target rectangle based on the outside edges of the particles.
    double outside_y = (pTop->topBound + pBottom->bottomBound) / 2.;
    double outside_x = (pLeft->leftBound + pRight->rightBound) / 2.;
    printf("outside y %g x %g\n", outside_y, outside_x);

    // Find the center of the target rectangle based on the center of the particles.
    double center_y = (pTop->yCenter + pBottom->yCenter + pLeft->yCenter + pRight->yCenter) / 4.;
    double center_x = (pTop->xCenter + pBottom->xCenter + pLeft->xCenter + pRight->xCenter) / 4.;
    printf("center  y %g x %g\n", center_y,  center_x);

    // Find the center of the target rectangle based on the inside edges of the particles.
    double inside_y = (pTop->bottomBound + pBottom->topBound) / 2.;
    double inside_x = (pLeft->rightBound + pRight->leftBound) / 2.;
    printf("inside  y %g x %g\n", inside_y,  inside_x);

    // Find the center of the target rectangle based on the center of the particles
    // that (should) lie on the *midlines* of the target.
    double midline_y = (pLeft->yCenter + pRight->yCenter) / 2.;
    double midline_x = (pTop->xCenter + pBottom->xCenter) / 2.;
    printf("midline y %g x %g\n", inside_y,  inside_x);

////////////////////////////////////

    //Constants used for calculation of position
    const double pi = 3.1415926535;
    const double FOV = pi / 4.;
    const double xResolution = 640.;
    const double yResolution = 480.;
    const double hoopWidthHalf = 39.375;

    // Ben's "pCenter" is my "midline_x"
    double pCenter = ((pTop->xCenter + pBottom->xCenter) / 2);

    // (positive or negative) angle between camera and target centerline
    double angle = ((xResolution / 2) - pCenter) * (FOV / xResolution);

    // (positive) angle between outer edge of left-hand target and target centerline
    double angleLeft = (pCenter - pLeft->leftBound) * (FOV / xResolution);

    // (positive) angle between target centerline and outer edge of right-hand target
    double angleRight = (pRight->rightBound - pCenter) * (FOV / xResolution);

    printf("values: %g, %g, %g\n", angleLeft, angleRight, angle);

    double leftmostAngle = (pi / 2.) - (angleLeft - angle);
    double rightmostAngle = (pi / 2.) - (angleRight - angle);
    double calcDistanceLeft = sin(leftmostAngle) * (hoopWidthHalf / sin(angleLeft));
    double calcDistanceRight = sin(rightmostAngle) * (hoopWidthHalf / sin(angleRight));
    double distance = (calcDistanceLeft + calcDistanceRight) / 2.;
    printf("distances: %g, %g %g\n", calcDistanceLeft, calcDistanceRight, distance);

    return true;
}
