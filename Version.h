// sample robot code
// Steve Tarr - team 1425 mentor

#ifndef _VERSION_H_
#define _VERSION_H_

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

class Version
{
public:
    Version( const char * ver );
    static const char * GetVersions();

private:
    static char * fileVersions;
    void AddVersion( const char * ver );
};

#endif // _VERSION_H_

