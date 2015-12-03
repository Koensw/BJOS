#ifndef _BLUEJAY_LOG_H_
#define _BLUEJAY_LOG_H_

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

#include <bjcomm/message.h>
#include <bjcomm/publisher.h>

/* 
 * Provides logging support while this has not yet been implemented
 */

//FIXME: add to BJOS namespace

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
    
    /*private static void _forward_to_gcs(const char *type, const char *origin, const char *format, ...){
        static Publisher pub("tcp://localhost:3334");
        if(!pub.isRunning()) pub.start();
        Message msg("logger");
        
        //FIXME: do this a little cleaner
        va_list args;
        fprintf( stdout, "[INFO] " );
        fprintf( stdout, "%s: ", origin);
        va_start( args, format );
        vfprintf( stdout, format, args );
        va_end( args );
        
        std::stringstream sstr;
        sstr << x << " " << y << " " << z;
        msg.setData(sstr.str());
        
        logger.setData("hier komt iets");
        pub.send(msg);
    }*/
};

#endif