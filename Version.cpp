// sample robot code
// Steve Tarr - team 1425 mentor

#include <stdLib.h>
#include <string.h>
#include "Version.h"

// Class Version helps keep track of the file versions
// of code installed on the robot.  Include Version.h
// in each C++ source file (NOT in headers!) and create
// a local file version number object e.g.
//   static Version v( __FILE__ " " __DATE__ " " __TIME__ );
// The Version class will collect the version numbers
// from each file when these objects are initialized.
// The combined report can then be displayed by e.g.
//   cout << Version::GetVersions();
// in your main robot constructor.

char * Version::fileVersions = NULL;

Version::Version( const char *ver )
{
    AddVersion(ver);
}

void Version::AddVersion( const char *ver )
{
    int len = strlen(ver) + 1;
    if (fileVersions == NULL) {
	fileVersions = (char *) malloc(len);
	strcpy(fileVersions, ver);
    } else {
	len += strlen(fileVersions) + 1;
	fileVersions = (char *) realloc(fileVersions, len);
	strcat(fileVersions, ver);
	strcat(fileVersions, "\n");
    }
}

const char * Version::GetVersions()
{
    return fileVersions;
}

static Version v( __FILE__ " " __DATE__ " " __TIME__ );
