// sample robot code
// Steve Tarr - team 1425 mentor - 25-Jan-2012

#include "Version.h"
#include <stdLib.h>
#include <string.h>

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
	strcat(fileVersions, "\n");
	strcat(fileVersions, ver);
    }
}

const char * Version::GetVersions()
{
    return fileVersions;
}

static Version v( __FILE__ " " __DATE__ " " __TIME__ );
