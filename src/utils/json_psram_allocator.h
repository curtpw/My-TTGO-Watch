#ifndef SPI_RAM_JSON_DOC_H_
#define SPI_RAM_JSON_DOC_H_

#include "config.h"
#include "ArduinoJson.h"
#include "alloc.h"

#ifdef NATIVE_64BIT
    #include "logging.h"
#endif

// arduinoJson allocator for external PSRAM
// see: https://arduinojson.org/v6/how-to/use-external-ram-on-esp32/
struct SpiRamAllocator {
    void* allocate( size_t size ) {
        void *ram = MALLOC( size );
        if ( ram ) {
            return( ram );
        }
        else {
            #ifdef NATIVE_64BIT
                log_e("allocate %ld bytes (%p) json psram failed", size, ram );
            #else
                log_e("allocate %d bytes (%p) json psram failed", size, ram );
            #endif
            if ( size == 0 ) {
                log_e("allocate zero bytes? really? abort");
            }
            return( ram );
        }
    }
    void deallocate( void* pointer ) {
        free( pointer );
    }
    void *reallocate( void *pointer, size_t size ) {
        void *ram = REALLOC( pointer, size );
        if ( ram ) {
            return( ram );
        }
        else {
            #ifdef NATIVE_64BIT
                log_e("rallocate %ld bytes (%p) json psram failed", size, ram );
            #else
                log_e("rallocate %d bytes (%p) json psram failed", size, ram );
            #endif
            if ( size == 0 ) {
                log_e("rallocate zero bytes? really? abort");
            }
            return( ram );
        }
    }
};
using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;

#endif