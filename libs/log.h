#ifndef _BLUEJAY_LOG_H_
#define _BLUEJAY_LOG_H_

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

/* 
 * Provides logging support while this has not yet been implemented
 */

class Log{
public:
    static void fatal(const char *origin, const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[FATAL] " );
        fprintf( stderr, "%s: ", origin);
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" ); 
    }
    
    static void warn(const char *origin, const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[WARN] " );
        fprintf( stderr, "%s: ", origin);
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" ); 
    }
    
    static void error(const char *origin, const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[ERROR] " );
        fprintf( stderr, "%s: ", origin);
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" ); 
    }
    
    static void info(const char *origin, const char* format, ... ) {
        va_list args;
        fprintf( stdout, "[INFO] " );
        fprintf( stdout, "%s: ", origin);
        va_start( args, format );
        vfprintf( stdout, format, args );
        va_end( args );
        fprintf( stdout, "\n" );
    }
};

#endif