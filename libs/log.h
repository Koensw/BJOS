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
    static void fatal( const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[FATAL] " );
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" ); 
        
        //FIXME: we should never want to randomly stop processes in a logger...
        std::exit(0);
    }
    
    static void warn( const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[WARN] " );
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" ); 
    }
    
    static void error( const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[ERROR] " );
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" ); 
    }
    
    static void info( const char* format, ... ) {
        va_list args;
        fprintf( stdout, "[INFO] " );
        va_start( args, format );
        vfprintf( stdout, format, args );
        va_end( args );
        fprintf( stdout, "\n" );
    }
};

#endif