// sample robot code
// Steve Tarr - team 1425 mentor - 11-Feb-2012

#ifndef _TARGET_H_
#define _TARGET_H_

#include <WPILib.h>
#include <vision/RGBImage.h>

class Target
{
public:
    Target();
    ~Target();

    bool GetImage();
    int FindParticles();
    bool AnalyzeParticles();

    void SaveImages();		// for debugging

private:
    // images
    RGBImage cameraImage;
    MonoImage monoImage;
    MonoImage equalized;
    MonoImage filtered;

    // particles
    struct Particle {
	int index;
	double size;
	double xCenter;
	double yCenter;
	double leftBound;
	double rightBound;
	double topBound;
	double bottomBound;
	double height;
	double width;
    } particles[4];

    Particle *pTop, *pBottom, *pLeft, *pRight;

    int num_particles;

    // processing constants
    ParticleFilterCriteria2 filterCriteria[1];
    ParticleFilterOptions2 filterOptions;
    RGBValue falseColor[256];
};

#endif // _TARGET_H_
