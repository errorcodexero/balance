
#ifndef _IDENT_H_
#define _IDENT_H_

class Version
{
public:
    Version( const char * ver );
    static const char * GetVersions();

private:
    static char * fileVersions;
    void AddVersion( const char * ver );
};

#endif // _IDENT_H_

