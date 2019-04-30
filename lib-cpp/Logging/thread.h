/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Thread.h
 * Author: Rboon
 *
 */

#ifndef THREAD_H
#define THREAD_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "structures.h"
#include "logging.h" 
#include <functional>

class Thread {
public:
    void* temp;
    typedef void (* vFunctionCall)(void);
    pthread_t thread1, thread2, thread3, thread4, thread5, thread6, thread7;

    Thread();
    Thread(const Thread& orig);
    
    virtual ~Thread();
    
    int CreateThreads();
    std::string getString();
    void writeControlData(structures::ControlData *control_data_ptr, bool to_logfile, bool to_python);
    void writeUserPower(structures::PowerInput *power_input_ptr, structures::PowerOutput *power_output_ptr, structures::UserInput *user_input_ptr, bool to_logfile, bool to_python);
    void writeTelemInput(structures::TelemetryInput *telemetry_input_ptr, bool to_logfile, bool to_python);
    //int CreateThread(pthread_t thread, vFunctionCall func);
    int CreatePipe(const char *path_to_pipe);
    
private:

};
#endif /* THREAD_H */

