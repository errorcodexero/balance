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

    typedef enum { kCenter, kLeft, kRight } TargetID;

    struct TargetLocation {
	TargetID id;		// which target
	double angle;		// in degrees, positive is to the right
	double distance;	// in inches
	bool valid;		// is the position data valid for targeting?
    };

    TargetLocation GetTarget( TargetID which );

private:
    // Target calculations take a long time,
    // so we run them in a background task.
    Task m_task;

    static void TargetProcess( Target *t );
    void Run();

    ////////////////////////////////////////////////////
    // Target locations returned to the main thread.
    // Accesses must be locked with the mutex.
    ////////////////////////////////////////////////////

    SEM_ID m_sem;
    TargetLocation m_targetLeft, m_targetRight, m_targetCenter;

    ///////////////////////////////////////////////////
    // The following variables are used by the
    // processing task and should not be accessed
    // from the main thread.
    ///////////////////////////////////////////////////

    // processing constants
    ParticleFilterCriteria2 m_filterCriteria[1];
    ParticleFilterOptions2 m_filterOptions;
    RGBValue m_falseColor[256];

    // images
    RGBImage m_cameraImage;
    MonoImage m_monoImage;
#if 0
    MonoImage m_equalized;
#endif
    MonoImage m_filtered;

    // particles (regions of interest)
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
    } m_particles[4];

    int m_numParticles;
    Particle *m_pTop, *m_pBottom, *m_pLeft, *m_pRight;
    bool m_topClipped, m_bottomClipped, m_leftClipped, m_rightClipped;

    // target locations
    double m_centerAngle;
    double m_centerDistance;
    double m_leftAngle;
    double m_leftDistance;
    double m_rightAngle;
    double m_rightDistance;
 
    // initialization
    void TargetInit();

    // targeting calculations
    bool GetImage();
    bool FindParticles();
    bool AnalyzeParticles();
    void SaveImages();
};

#endif // _TARGET_H_
