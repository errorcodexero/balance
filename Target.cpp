// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#include <WPILib.h>
#include <math.h>
#include "Target.h"
#include "Version.h"

static Version v( __FILE__ " " __DATE__ " " __TIME__ );

void Target::TargetProcess( Target *t )
{
    t->Run();
}

Target::Target() :
    m_task( "TargetProcess", (FUNCPTR)TargetProcess )
{
    TargetInit();
}

void Target::TargetInit()
{
    memset(m_falseColor, 0, sizeof(m_falseColor));
    for (int i = 1; i < 255; i++) {
	switch (i % 6) {
	case 0:	// white
	    m_falseColor[i].R = m_falseColor[i].G = m_falseColor[i].B = 255;
	    break;
	case 1: // magenta
	    m_falseColor[i].B = m_falseColor[i].R = 255;
	    break;
	case 2: // red
	    m_falseColor[i].R = 255;
	    break;
	case 3: // yellow
	    m_falseColor[i].R = m_falseColor[i].G = 255;
	    break;
	case 4: // green
	    m_falseColor[i].G = 255;
	    break;
	case 5: // cyan
	    m_falseColor[i].G = m_falseColor[i].B = 255;
	    break;
	}
    }

    m_filterCriteria[0].parameter  = IMAQ_MT_PARTICLE_AND_HOLES_AREA;
    m_filterCriteria[0].lower      = 0;
    m_filterCriteria[0].upper      = 400;
    m_filterCriteria[0].calibrated = FALSE;
    m_filterCriteria[0].exclude    = TRUE;

    m_filterOptions.rejectMatches = FALSE;
    m_filterOptions.rejectBorder  = FALSE;
    m_filterOptions.fillHoles     = TRUE;
    m_filterOptions.connectivity8 = TRUE;

    m_targetCenter.id = kCenter;
    m_targetCenter.angle = 0.0;
    m_targetCenter.distance = 0.0;
    m_targetCenter.valid = false;

    m_targetTop.id = kTop;
    m_targetTop.angle = 0.0;
    m_targetTop.distance = 0.0;
    m_targetTop.valid = false;

    m_targetBottom.id = kBottom;
    m_targetBottom.angle = 0.0;
    m_targetBottom.distance = 0.0;
    m_targetBottom.valid = false;

    m_targetLeft.id = kLeft;
    m_targetLeft.angle = 0.0;
    m_targetLeft.distance = 0.0;
    m_targetLeft.valid = false;

    m_targetRight.id = kRight;
    m_targetRight.angle = 0.0;
    m_targetRight.distance = 0.0;
    m_targetRight.valid = false;
}

Target::~Target()
{
}

Target::TargetLocation Target::GetTarget( TargetID which )
{
    TargetLocation target;
    switch (which) {
    case kCenter:
	{
	    Synchronized mutex(m_sem);
	    target = m_targetCenter;
	}
	break;
    case kTop:
	{
	    Synchronized mutex(m_sem);
	    target = m_targetTop;
	}
	break;
    case kBottom:
	{
	    Synchronized mutex(m_sem);
	    target = m_targetBottom;
	}
	break;
    case kLeft:
	{
	    Synchronized mutex(m_sem);
	    target = m_targetLeft;
	}
	break;
    case kRight:
	{
	    Synchronized mutex(m_sem);
	    target = m_targetRight;
	}
	break;
    default:
	target.id = which;
	target.angle = 0.0;
	target.distance = 0.0;
	target.valid = false;
    }

    return target;
}

//////////////////////////////////////////////////////////////////////////////
//
// image processing constants
//
//////////////////////////////////////////////////////////////////////////////

// IMAGE_PLANE is the distance to the image plane in pixel-width units:
// d' = (half of image width) / tan(half of horizontal field-of-view angle)
//
// from the Axis 206 data sheet:
// image sensor: 1/4" (diagonal)
// lens: 4.0mm, F2.0
// horizontal FOV: 54 degrees
//
// from the Axis M1011 data sheet:
// image sensor: 1/4" (diagonal)
// lens: 4.4mm, F2.0
// horizontal FOV: 47 degrees
//
// from Axis's on-line calculator
// and camera comparison page:
// M1011 lens: 4.2mm F2.0
//
// These numbers don't make sense!  The stated FOV is much smaller than
// that calculated from the lens FL and sensor size.  The FOVs for the
// two cameras are very different, even though they supposedly use the
// same size sensor and only slightly different lenses.
//
// For now, assume horizontal angle = 49 degrees and WIDTH = 640 pixels
// tan(h/2) = tan(24.5 degrees) = 0.45572626
// (WIDTH/2) / tan(h/2) = 702.17591

#define	WIDTH	640	// image width, in pixels
#define	HEIGHT	480	// image height, in pixels
#define	BORDER	4	// expected minimum spacing from objects to edge of image

#define	IMAGE_PLANE 702.17591

// FRC target dimensions:
//
// hoop vertical spacing
// 28", 61", 98" from floor to top of rim
//
// hoop horizontal spacing
// midline of top/bottom hoop to midline of side hoops 27 3/8"
// midline of left hoop to midline of right hoop 54 3/4"
//
// backboard: 44" x 31 1/2" outside, with dark 2" border
// reflective target: 24" x 18" outside, 2" reflective stripe
// (so inside dimensions are 20" x 14")
// there is an additional 2" dark border inside and outside the reflective stripe
// the bottom edge of the reflective area is 2" above the hoop rim
//
// midline of top/bottom reflector to midline of side reflector 27 3/8"
//
// midline of top/bottom reflector to nearest outer edge of side reflector
// 27 3/8" - (24" / 2) = 15 3/8"
//
// midline of top/bottom reflector to farthest outer edge of side reflector
// 27 3/8" + (24" / 2) = 39 3/8"

#define	FRC_WIDTH	12.000		// half the width of the reflector
#define	FRC_INSIDE	15.375		// center to nearest edge of reflector
#define	FRC_MIDLINE	27.375		// center to midline of reflector
#define	FRC_OUTSIDE	39.375		// center to farthest edge of reflector

// trig constants

#ifndef M_PI
#define	M_PI	3.1415926535
#endif

#define	DEGREES	(180./M_PI)
#define	RADIANS	(M_PI/180.)

//////////////////////////////////////////////////////////////////////////////
//
// image processing functions
//
// These are called from Target::Run to handle all image processing and
// target detection operations.  The variables that they use should not be
// accessed outside the background task.  Target::Run will copy the
// processing results to the TargetLocation variables that are accessible
// from the main task.
//
//////////////////////////////////////////////////////////////////////////////

void Target::Run()
{
    while (1) {
	if (GetImage()) {
	    if (FindParticles() && AnalyzeParticles()) {
		// store the analysis results
		{
		    Synchronized mutex(m_sem);

		    m_targetCenter.angle = m_centerAngle;
		    m_targetCenter.distance = m_centerDistance;
		    m_targetCenter.valid = true;

		    m_targetTop.angle = m_topAngle;
		    //m_targetTop.distance = m_topDistance;
		    m_targetTop.distance = m_centerDistance;
		    m_targetTop.valid = true;

		    m_targetBottom.angle = m_bottomAngle;
		    //m_targetBottom.distance = m_bottomDistance;
		    m_targetBottom.distance = m_centerDistance;
		    m_targetBottom.valid = true;

		    m_targetLeft.angle = m_leftAngle;
		    m_targetLeft.distance = m_leftDistance;
		    m_targetLeft.valid = !m_leftClipped;

		    m_targetRight.angle = m_rightAngle;
		    m_targetRight.distance = m_rightDistance;
		    m_targetRight.valid = !m_rightClipped;
		}
	    } else {
		{
		    Synchronized mutex(m_sem);

		    m_targetCenter.valid = false;
		    m_targetTop.valid = false;
		    m_targetBottom.valid = false;
		    m_targetLeft.valid = false;
		    m_targetRight.valid = false;
		}
	    }
	    SaveImages();
	    Wait(4.0);		// debugging aid; remove when this is working
	}
    }
}

bool Target::GetImage()
{
    long then = (long) GetFPGATime();

    AxisCamera& axisCamera = AxisCamera::GetInstance();
    if (!axisCamera.IsFreshImage()) {
	return false;
    }
    if (!axisCamera.GetImage(&m_cameraImage)) {
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

    if (!imaqWriteFile(m_cameraImage.GetImaqImage(), "/tmp/vision00-camera.bmp", NULL)) {
	printf("%s: imaqWriteFile(\"/tmp/vision00-camera.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }
    if (!imaqWriteFile(m_monoImage.GetImaqImage(), "/tmp/vision01-monoImage.bmp", NULL)) {
	printf("%s: imaqWriteFile(\"/tmp/vision01-monoImage.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }
#if 0
    if (!imaqWriteFile(m_equalized.GetImaqImage(), "/tmp/vision02-equalized.bmp", NULL)) {
	printf("%s: imaqWriteFile(\"/tmp/vision02-equalized.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }
#endif
    if (!imaqWriteFile(m_filtered.GetImaqImage(), "/tmp/vision03-filtered.bmp", m_falseColor)) {
	printf("%s: imaqWriteFile(\"/tmp/vision03-filtered.bmp\") FAILED\n", __FUNCTION__);
	// ignore the error
    }

    long now = (long) GetFPGATime();
    printf("%s: image save took %ld microseconds\n", __FUNCTION__, (now - then));
}

bool Target::FindParticles()
{
    long then = (long) GetFPGATime();
    MonoImage *pImage = NULL;

    // extract the blue plane
    if (!imaqExtractColorPlanes(m_cameraImage.GetImaqImage(), IMAQ_RGB,
    		NULL, NULL, m_monoImage.GetImaqImage()))
    {
	printf("%s: imaqExtractColorPlanes FAILED\n", __FUNCTION__);
	return false;
    }
    pImage = &m_monoImage;

#if 0
    // apply auto-median to simplify particles
    if (!imaqGrayMorphology(m_equalized.GetImaqImage(), m_monoImage.GetImaqImage(),
    		IMAQ_AUTOM, NULL))
    {
	printf("%s: imaqGrayMorphology FAILED\n", __FUNCTION__);
	return false;
    }
    pImage = &m_equalized;
#endif

    // select interesting particles
    int particleCount = 0;
    if (!imaqParticleFilter4(m_filtered.GetImaqImage(), pImage->GetImaqImage(),
    		m_filterCriteria, 1, &m_filterOptions, NULL, &particleCount))
    {
	printf("%s: imaqParticleFilter FAILED\n", __FUNCTION__);
	return false;
    }

    // select the four largest particles (insertion sort)
    // for now, keep track of only the particle number (index) and size
    memset((void *)m_particles, 0, sizeof m_particles);
    for (int i = 0; i < particleCount; i++) {
	double size;
	if (!imaqMeasureParticle(m_filtered.GetImaqImage(), i, FALSE,
	    IMAQ_MT_PARTICLE_AND_HOLES_AREA, &size))
	{
	    printf("%s: imaqMeasureParticle %d FAILED\n", __FUNCTION__, i);
	    break;
	}
	for (int j = 0; j < 4; j++) {
	    if (size > m_particles[j].size) {
		for (int k = 3; k > j; k--) {
		    m_particles[k].index = m_particles[k-1].index;
		    m_particles[k].size = m_particles[k-1].size;
		}
		m_particles[j].index = i;
		m_particles[j].size = size;
		break;
	    }
	}
    }

    // fill in the rest of the measured data
    for (m_numParticles = 0;
    	 m_numParticles < 4 && m_particles[m_numParticles].size > 0;
	 m_numParticles++)
    {
	Particle* p = &m_particles[m_numParticles];
	imaqMeasureParticle(m_filtered.GetImaqImage(), p->index, FALSE,
			    IMAQ_MT_CENTER_OF_MASS_X, &(p->xCenter));
	imaqMeasureParticle(m_filtered.GetImaqImage(), p->index, FALSE,
			    IMAQ_MT_CENTER_OF_MASS_Y, &(p->yCenter));
	imaqMeasureParticle(m_filtered.GetImaqImage(), p->index, FALSE,
			    IMAQ_MT_BOUNDING_RECT_LEFT, &(p->leftBound));
	imaqMeasureParticle(m_filtered.GetImaqImage(), p->index, FALSE,
			    IMAQ_MT_BOUNDING_RECT_RIGHT, &(p->rightBound));
	imaqMeasureParticle(m_filtered.GetImaqImage(), p->index, FALSE,
			    IMAQ_MT_BOUNDING_RECT_TOP, &(p->topBound)); 
	imaqMeasureParticle(m_filtered.GetImaqImage(), p->index, FALSE,
			    IMAQ_MT_BOUNDING_RECT_BOTTOM, &(p->bottomBound));
	p->height = p->bottomBound - p->topBound;
	p->width = p->rightBound - p->leftBound;
    }

    long now = (long) GetFPGATime();
    printf("%s: particle detection took %ld microseconds\n", __FUNCTION__, (now - then));
    printf("%s: found %d particles\n", __FUNCTION__, particleCount);
    printf("%s: returning %d particles\n", __FUNCTION__, m_numParticles);
    for (int i = 0; i < m_numParticles; i++) {
	Particle *p = &m_particles[i];
	printf("  particle %d top %g bottom %g left %g right %g size %g x %g y %g\n",
		p->index, p->topBound, p->bottomBound, p->leftBound, p->rightBound,
		p->size, p->xCenter, p->yCenter);
    }

    return true;
}

bool Target::AnalyzeParticles()
{
    long then = (long) GetFPGATime();

    if (m_numParticles < 3) {
	printf("ERROR: m_numParticles < 3, no analysis possible\n");
	return false;
    }

    // image quality tests:
    //
    // 1) The particles should be laid out without overlapping.  For example, the left
    // and right bounds of the top particle should not overlap the right bound of the
    // left particle or the left bound of the right particle.  This relationship can
    // be used to determine which particle is missing when only 3 are in view.
    // (If only 2 particles are in view, we can't use this to distinguish between e.g.
    // the top+left particles and the right+bottom particles.  TBD: see if we can use
    // e.g. absolute height information to sort this out when we're really close to
    // the targets.)
    //
    // 2) There should be some space between the outer edge of each particle and the edge
    // of the image.  If not, one of the particles is partially outside the field of view.
    //
    // 3) The four particles should be approximately the same size.  If not, one or more
    // of the particles is partially hidden or we didn't identify the particles correctly.
    //
    // 4) The aspect ratio (height to width ratio) of each (unclipped) particle should be
    // approximately that of the target rectangles.
    //
    // 5) The image center position calculated from the inside edges of the particles,
    // the center of mass of the particles, and the outside edges of the particles should
    // be approximately the same.

    // Sort the particles by position, based on their inside edges.
    // These are in image coordinates, so (0,0) is top-left.

    m_pTop = m_pBottom = m_pLeft = m_pRight = &m_particles[0];

    for (int i = 1; i < m_numParticles; i++) {
	Particle *p = &m_particles[i];
	if (p->bottomBound < m_pTop->bottomBound) {
	    m_pTop = p;
	}
	if (p->topBound > m_pBottom->topBound) {
	    m_pBottom = p;
	}
	if (p->rightBound < m_pLeft->rightBound) {
	    m_pLeft = p;
	}
	if (p->leftBound > m_pRight->leftBound) {
	    m_pRight = p;
	}
    }

    printf("top %d bottom %d left %d right %d\n",
	m_pTop->index, m_pBottom->index, m_pLeft->index, m_pRight->index);

    if (m_numParticles < 4) {
	// TBD: The if-then-elseif structure here assumes that only one
	// particle is missing.  Rework this to deal with two, or even three
	// missing particles.
	if (m_pTop->bottomBound > m_pLeft->topBound ||
	    m_pTop->bottomBound > m_pRight->topBound)
	{
	    m_pTop = NULL;
	}
	else if (m_pLeft->rightBound > m_pTop->leftBound ||
	    m_pLeft->rightBound > m_pBottom->leftBound)
	{
	    m_pLeft = NULL;
	}
	else if (m_pRight->leftBound < m_pTop->rightBound ||
	    m_pRight->leftBound < m_pBottom->rightBound)
	{
	    m_pRight = NULL;
	}
	else if (m_pBottom->topBound < m_pLeft->bottomBound ||
	    m_pBottom->topBound < m_pRight->bottomBound)
	{
	    m_pBottom = NULL;
	}
	else
	{
	    printf("ERROR: particle overlap can't be resolved\n");
	    return false;
	}
    }

    // uniqueness check
    if (m_pTop == m_pLeft || m_pTop == m_pRight || m_pTop == m_pBottom ||
	m_pLeft == m_pRight || m_pLeft == m_pBottom || m_pRight == m_pBottom) {
	printf("ERROR: particles aren't unique.\n");
	return false;
    }

    // check outside boundaries and particle sizes for clipping and image artifacts
    m_topClipped = m_bottomClipped = m_leftClipped = m_rightClipped = false;

    float size; // median size
    if (m_numParticles == 4) {
	size = (m_particles[1].size + m_particles[2].size) / 2.;
    } else {
	// assume 3 particles
	size = m_particles[1].size;
    }
    // These limits are very large in order to accomodate
    // image distortions from the hoops and nets at close range.
    float min_size = size * 0.50;  // 1/2 of median
    float max_size = size * 2.00;  // 5/4 of median

    if (m_pTop) {
	if (m_pTop->topBound < BORDER) {
	    printf("WARNING: top particle is clipped\n");
	    m_topClipped = true;
	}
	else if (m_pTop->size < min_size) {
	    // should this have been caught by the particle filter?
	    printf("WARNING: top particle is unreasonably small\n");
	    m_pTop = NULL;
	    m_numParticles--;
	    m_topClipped = true;
	}
	else if (m_pTop->size > max_size) {
	    // should this have been caught by the particle filter?
	    printf("WARNING: top particle is unreasonably large\n");
	    m_pTop = NULL;
	    m_numParticles--;
	    m_topClipped = true;
	}
    } else {
	m_topClipped = true;
    }

    if (m_pBottom) {
	if (m_pBottom->bottomBound > (HEIGHT - BORDER)) {
	    printf("WARNING: bottom particle is clipped\n");
	    m_bottomClipped = true;
	}
	else if (m_pBottom->size < min_size) {
	    printf("WARNING: bottom particle is unreasonably small\n");
	    m_pBottom = NULL;
	    m_numParticles--;
	    m_bottomClipped = true;
	}
	else if (m_pBottom->size > max_size) {
	    printf("WARNING: bottom particle is unreasonably large\n");
	    m_pBottom = NULL;
	    m_numParticles--;
	    m_bottomClipped = true;
	}
    } else {
	m_bottomClipped = true;
    }

    if (m_pLeft) {
	if (m_pLeft->leftBound < BORDER) {
	    printf("WARNING: left particle is clipped\n");
	    m_leftClipped = true;
	}
	else if (m_pLeft->size < min_size) {
	    printf("WARNING: left particle is unreasonably small\n");
	    m_pLeft = NULL;
	    m_numParticles--;
	    m_leftClipped = true;
	}
	else if (m_pLeft->size > max_size) {
	    printf("WARNING: left particle is unreasonably large\n");
	    m_pLeft = NULL;
	    m_numParticles--;
	    m_leftClipped = true;
	}
    } else {
	m_leftClipped = true;
    }

    if (m_pRight) {
	if (m_pRight->rightBound > (WIDTH - BORDER)) {
	    printf("WARNING: right particle is clipped\n");
	    m_rightClipped = true;
	}
	else if (m_pRight->size < min_size) {
	    printf("WARNING: right particle is unreasonably small\n");
	    m_pRight = NULL;
	    m_numParticles--;
	    m_rightClipped = true;
	}
	else if (m_pRight->size > max_size) {
	    printf("WARNING: right particle is unreasonably large\n");
	    m_pRight = NULL;
	    m_numParticles--;
	    m_rightClipped = true;
	}
    } else {
	m_rightClipped = true;
    }

    if (m_numParticles < 3) {
	printf("ERROR: not enough particles remaining for analysis\n");
	return false;
    }

    //////////////////////////////////////////////////////////////////////////
    //
    // Calculate the angle and distance to the center of the target(s).
    //
    // Project data from the image plane to the real-world plane of the
    // backboards using what we know about the real-world dimensions of
    // the features we're seeing.  From that, we can calculate the angle
    // and distance from the camera centerline to each feature.  In order
    // to aim the robot, we may also need to deal with a rotation and/or
    // translation between the camera centerline and the robot centerline,
    // but we'll ignore that for now.
    //
    // If we're shooting from either side of the field, we may need to
    // adjust our aiming point to hit the center of the hoops rather than
    // aiming directly for the center of the reflective targets.  Again,
    // we'll ignore that for now.
    //
    // For a given field-of-view angle and image plane width (in pixels),
    // the distance (in pixel-width units) to the image plane is given by:
    //   d' = (w'/2) / tan(h/2)
    // where w is the width of the image plane in pixels (640) and h is
    // the horizontal angle of view (49 degrees for this camera/lens.)
    // This constant has been pre-calculated (IMAGE_PLANE).
    //
    // The view angle for an object at x' pixels from the center can be
    // solved from:
    //   d' = x' / tan(a)
    //   tan(a) = x' / d'
    //   a = atan(x' / d')
    //
    // Given a desired target and the real-world distances to visible targets
    // on either side of it, we can calculate the distance to the target.
    // Once we have the distance to the center target, we can calculate
    // the distances to the targets on either side.
    //
    //////////////////////////////////////////////////////////////////////////

    if (m_pTop) {
	m_topAngle = atan((m_pTop->xCenter-(WIDTH/2))/IMAGE_PLANE)*DEGREES;
	printf("top angle %g degrees\n", m_topAngle);
    }

    if (m_pBottom) {
	m_bottomAngle = atan((m_pBottom->xCenter-(WIDTH/2))/IMAGE_PLANE)*DEGREES;
	printf("bottom angle %g degrees\n", m_bottomAngle);
    }

    double centerX;
    if (m_pTop && m_pBottom) {
	centerX = (m_pTop->xCenter + m_pBottom->xCenter) / 2.;
    } else if (m_pTop) {
	centerX = m_pTop->xCenter;
    } else if (m_pBottom) {
	centerX = m_pBottom->xCenter;
    } else {
	printf("ERROR: m_pTop and m_pBottom are both NULL (can't happen!)\n");
	return false;
    }

    m_centerAngle = atan((centerX - (WIDTH/2)) / IMAGE_PLANE) * DEGREES;
    printf("center angle %g degrees\n", m_centerAngle);

    double leftWidthReal;
    double leftWidthImage;
    if (m_pLeft) {
	if (m_leftClipped) {
	    leftWidthReal = FRC_INSIDE;
	    leftWidthImage = centerX - m_pLeft->rightBound;
	} else {
	    leftWidthReal = FRC_OUTSIDE;
	    leftWidthImage = centerX - m_pLeft->leftBound;
	}
    } else if (m_pTop) {
	leftWidthReal = FRC_WIDTH;
	leftWidthImage = centerX - m_pTop->leftBound;
    } else if (m_pBottom) {
	leftWidthReal = FRC_WIDTH;
	leftWidthImage = centerX - m_pBottom->leftBound;
    } else {
	printf("ERROR: m_pTop and m_pBottom are both NULL (can't happen!)\n");
	return false;
    }

    double leftAngle = atan( leftWidthImage / IMAGE_PLANE );
    m_leftAngle = leftAngle * DEGREES;
    printf("left angle %g degrees\n", m_leftAngle);

    double rightWidthReal;
    double rightWidthImage;

    if (m_pRight) {
	if (m_rightClipped) {
	    rightWidthReal = FRC_INSIDE;
	    rightWidthImage = m_pRight->leftBound - centerX;
	} else {
	    rightWidthReal = FRC_OUTSIDE;
	    rightWidthImage = m_pRight->rightBound - centerX;
	}
    } else if (m_pTop) {
	rightWidthReal = FRC_WIDTH;
	rightWidthImage = m_pTop->rightBound - centerX;
    } else if (m_pBottom) {
	rightWidthReal = FRC_WIDTH;
	rightWidthImage = m_pBottom->rightBound - centerX;
    } else {
	printf("ERROR: m_pTop and m_pBottom are both NULL (can't happen!)\n");
	return false;
    }

    double rightAngle = atan( rightWidthImage / IMAGE_PLANE );
    m_rightAngle = rightAngle * DEGREES;
    printf("right angle %g degrees\n", m_rightAngle);

    // these quantities appear repeatedly in the calculations
    double known1 = sin(leftAngle) / leftWidthReal;
    double known2 = sin(rightAngle) / rightWidthReal;
    double known3 = M_PI - leftAngle - rightAngle;
    double sin3 = sin(known3);
    double cos3 = cos(known3);

    // Calculate the distance to the center (c) using either
    // the triangle on the left or the right.
    double alpha = atan((known1*sin3)/(known2+known1*cos3));
    double c1 = sin(alpha) / known1;

    double beta = atan((known2*sin3)/(known1+known2*cos3));
    double c2 = sin(beta) / known2;

    // Check: these two distance calculations should agree to within
    // the floating-point calculation precision.
    if (fabs(c2-c1) > (c1+c2)*0.0001) {
	printf("ERROR: calculated distances do not agree (can't happen)\n");
	printf("distance1 %g, distance2 %g\n", c1, c2);
	return false;
    }

    m_centerDistance = c1;

    // angle "gamma1" and side "b" in Jeff's diagram
    m_leftDistance = sin(M_PI - leftAngle - alpha) / known1;

    // angle "gamma2" and side "a" in Jeff's diagram
    m_rightDistance = sin(M_PI - leftAngle - beta) / known1;

    printf("distance left %g, center %g, right %g\n",
	    m_leftDistance, m_centerDistance, m_rightDistance);

    long now = (long) GetFPGATime();
    printf("%s: particle analysis took %ld microseconds\n", __FUNCTION__, (now - then));

    return true;
}

