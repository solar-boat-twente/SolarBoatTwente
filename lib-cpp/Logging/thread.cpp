/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Thread.cpp
 * Author: Rboon
 * 
 * Created on 23 mei 2018, 11:06
 */

#include "thread.h"
#define CONTROL_DATA_WRITE_CYCLE_TIME 13333
#define USER_POWER_WRITE_CYCLE_TIME 200000
#define TELEM_INPUT_WRITE_CYCLE_TIME 20000

using namespace MIO;

Thread::Thread()
{
    // path to pipes
    /*Logger user_power_logger ("config/user_power.conf");
    structures::PowerInput *power_input_ptr, power_input;
    power_input_ptr = &power_input;
    structures::PowerOutput *power_output_ptr, power_output;
    power_output_ptr = &power_output;
    structures::UserInput *user_input_ptr, user_input;
    user_input_ptr = &user_input;
     * */
}

Thread::Thread(const Thread& orig)
{
    
}

Thread::~Thread()
{
    pthread_join(thread1, &temp);
    pthread_join(thread2, &temp);
    pthread_join(thread3, &temp);
    pthread_join(thread4, &temp);
}

//const char *pipe_control_data_to_python = "/root/SolarBoat 2019/SolarBoatTwente/pipes/ControlDataToPython";
//const char *pipe_user_power_to_python = "/root/SolarBoat 2019/SolarBoatTwente/pipes/UserPowerToPython";
//const char *pipe_telem_input_to_python = "/root/SolarBoat 2019/SolarBoatTwente/pipes/TelemInputToPython";

const char *pipe_control_data_to_python = "/root/SolarBoat2019/SolarBoatTwente/pipes/ControlDataToPython";
const char *pipe_user_power_to_python = "/root/SolarBoat2019/SolarBoatTwente/pipes/UserPowerToPython";
const char *pipe_telem_input_to_python = "/root/SolarBoat2019/SolarBoatTwente/pipes/TelemInputToPython";

Logger control_data_logger("/root/SolarBoat2019/SolarBoatTwente/config/control_data.conf");
Logger user_power_logger("/root/SolarBoat2019/SolarBoatTwente/config/user_power.conf");
Logger telem_input_logger("/root/SolarBoat2019/SolarBoatTwente/config/telem_input.conf");

Logger control_data_pipe("/root/SolarBoat2019/SolarBoatTwente/config/control_data_python.conf");
Logger user_power_pipe("/root/SolarBoat2019/SolarBoatTwente/config/user_power_python.conf");
Logger telem_input_pipe("/root/SolarBoat2019/SolarBoatTwente/config/telem_input_python.conf");

structures::ControlData *gl_control_data_ptr;
structures::PowerInput *gl_power_input_ptr;
structures::PowerOutput *gl_power_output_ptr;
structures::UserInput *gl_user_input_ptr;
structures::TelemetryInput *gl_telemetry_input_ptr;
  

//prototype functions
void *ThreadReadPipe(void *ptr);
void *ThreadWriteControlData(void *ptr);
void *ThreadWriteUserPowerData(void *ptr);
void *ThreadWriteTelemInputData(void *ptr);
//int CreatePipe(const char *path_to_pipe);
//int CreateThread(pthread_t thread, std::function<void(void)> func);
inline bool exist_test(const char * name_char);

int flagControlDataThread = 0;
int flagUserPowerThread = 0;
int flagTelemThread = 0;

bool ControlDataToLogfile = 0;
bool ControlDataToPython = 0;
bool UserPowerToLogfile = 0;
bool UserPowerToPython = 0;
bool TelemInputToLogfile = 0;
bool TelemInputToPython = 0;

//create mutexes  
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex3 = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex4 = PTHREAD_MUTEX_INITIALIZER;



/*
int Thread::CreateThread(pthread_t thread, vFunctionCall func)
{
    pthread_create(&thread, NULL, func(), NULL);
    if (errno == 0)
    {
        std::cout << "Thread created succesfully" << std::endl;
    }
    else
    {
        std::cout << "Creating thread  failed! Errno: " << errno << ": ";
        std::cout << strerror(errno) << std::endl;
        //return -1;
    }
}
 * */
int Thread::CreateThreads()
{
    /*! \fn int Thread::CreateThread()
     \brief Creates a thread for reading and writing via pipes to server
     \return 0 if succesfull, returns -1 if failed 
     */
    CreatePipe(pipe_control_data_to_python);
    CreatePipe(pipe_user_power_to_python);    
    CreatePipe(pipe_telem_input_to_python);
       
    /*
    pthread_create(&thread1, NULL, ThreadReadPipe, NULL);
    if (errno == 0)
    {
        std::cout << "Thread "<< pipe_control_data_to_python<<" created succesfully" << std::endl;
    }
    else
    {
        std::cout << "Creating thread "<<pipe_control_data_to_python <<"  failed! Errno: " << errno << ": ";
        std::cout << strerror(errno) << std::endl;
    }
     * */
    pthread_create(&thread2, NULL, ThreadWriteControlData, NULL);
    if (errno == 0)
    {
        std::cout << "Thread "<<pipe_control_data_to_python <<" created succesfully" << std::endl;
    }
    else
    {
        std::cout << "Creating thread for "<<pipe_control_data_to_python <<"  failed! Errno: " << errno << ": ";
        std::cout << strerror(errno) << std::endl;
    }
    
    pthread_create(&thread3, NULL, ThreadWriteUserPowerData, NULL);
    if (errno == 0)
    {
        std::cout << "Thread "<< pipe_user_power_to_python <<" created succesfully" << std::endl;
    }
    else
    {
        std::cout << "Creating thread for "<< pipe_user_power_to_python <<" failed! Errno: " << errno << ": ";
        std::cout << strerror(errno) << std::endl;
    }
    pthread_create(&thread4, NULL, ThreadWriteTelemInputData, NULL);
    if (errno == 0)
    {
        std::cout << "Thread "<< pipe_telem_input_to_python<< " created succesfully" << std::endl;
    }
    else
    {
        std::cout << "Creating thread for "<< pipe_telem_input_to_python<< " failed! Errno: " << errno << ": ";
        std::cout << strerror(errno) << std::endl;
    }
     
    return 0;
}


int Thread::CreatePipe(const char *path_to_pipe)
{
    bool ExistFiFo = exist_test(path_to_pipe);
    if (ExistFiFo == 0)
    {  
        if (mkfifo(path_to_pipe, 0666))
        {
            std::cout << "Pipe: " << path_to_pipe  <<  "created succesfully" << std::endl;
        }
        else
        {
            std::cout << "Creating pipe: " << path_to_pipe  <<" failed! Errno: " << errno << ": ";
            std::cout << strerror(errno) << std::endl;
        }
    }
}

std::string Thread::getString()
{
    /*! \fn void Thread::getString(char* message)
    \brief for getting a array of chars from the thrread to the main to use there
    \param message contains a array of chars that can be transfered to the main through a pointer
     */

    //std::string UntilDelim;
    
    //std::istringstream InputStringStream(InputString);
    //std::getline(InputStringStream, UntilDelim, '$'); // gets line until stop sign '$'
    //memset(InputString, '\0', sizeof (InputString));

   //return UntilDelim;
}

void Thread::writeControlData(structures::ControlData *control_data_ptr, bool to_logfile, bool to_python)
{
    /*! \fn void Thread::writeString(char* message, int flagWrite)
      \brief for writing messages, from the main through the thread to the server
      \param flagWrite sets a flag in the thread if there needs to be data written to the server. Keeps the thread from spamming the server side.
     */

    gl_control_data_ptr = control_data_ptr;
    ControlDataToLogfile = to_logfile;
    ControlDataToPython = to_python;
    
}

void Thread::writeUserPower(structures::PowerInput *power_input_ptr, structures::PowerOutput *power_output_ptr, structures::UserInput *user_input_ptr, bool to_logfile, bool to_python)
{
    /*! \fn void Thread::writeString(char* message, int flagWrite)
      \brief for writing messages, from the main through the thread to the server
      \param flagWrite sets a flag in the thread if there needs to be data written to the server. Keeps the thread from spamming the server side.
     */

    gl_power_input_ptr = power_input_ptr;
    gl_power_output_ptr = power_output_ptr;
    gl_user_input_ptr = user_input_ptr;
    UserPowerToLogfile = to_logfile;
    UserPowerToPython = to_python;
}

void Thread::writeTelemInput(structures::TelemetryInput *telemetry_input_ptr, bool to_logfile, bool to_python)
{
    /*! \fn void Thread::writeString(char* message, int flagWrite)
      \brief for writing messages, from the main through the thread to the server
      \param flagWrite sets a flag in the thread if there needs to be data written to the server. Keeps the thread from spamming the server side.
     */

    gl_telemetry_input_ptr = telemetry_input_ptr;
    TelemInputToLogfile = to_logfile;
    TelemInputToPython = to_python;
}






void *ThreadReadPipe(void *ptr)
{
    /*! \fn void *ThreadReadPipe(void *ptr)
     \brief This is a thread to open the pipe, read data, and closes it every 10000 microseconds
     \param *ptr for making thread
     */
/*
    int FifoA;
    char tempString[1000]; //= {};
    while (1)
    {
        memset(tempString, '\0', sizeof (tempString));
        pthread_mutex_lock(&mutex1);
        FifoA = open(PathToPipeA, O_RDONLY);
        std::cout << "reading" << std::endl;
        read(FifoA, tempString, 1000);
        close(FifoA);
        strcpy(InputString, tempString);
        pthread_mutex_unlock(&mutex1);
        std::cout << "String: " << InputString << std::endl;
        if (errno == 0)
        {
            std::cout << "Fifo A opening/read succesfully" << std::endl;
        }
        else
        {
            std::cout << "Reading/opening Fifo A failed! Errno: " << errno << ": ";
            std::cout << strerror(errno) << std::endl;
            //return -1;
        }
        usleep(100000);
    }*/
}


void *ThreadWriteControlData(void *ptr)
{
    /*! \fn void *ThreadWritePipe(void *ptr)
     \brief This is a thread to open the pipe, write data, and closes it every 10000 microseconds
     \param *ptr for making thread
     */
    int Fifo;
    while (1)
    {
        if (ControlDataToLogfile == 1) // only needs to write if the command is given
        {
            //pthread_mutex_lock(&mutex2);
            //std::cout << "Writing to  log now" << std::endl;
            //freopen("/root/SolarBoat2019/SolarBoatTwente/logfiles/ControlData.log", "w", stdout);
            //std::cout << "TESTESTETS" << std::endl;
            control_data_logger.write_struct_control_data(gl_control_data_ptr);
             //fclose(stdout);
            //freopen("/root/SolarBoat2019/SolarBoatTwente/logfiles/ControlData.log", "w", stdout);
            //std::cout << "JOEHOE" << std::endl;
            //fclose(stdout);
            ControlDataToLogfile = 0;
           
            //pthread_mutex_unlock(&mutex2);
            if (errno == 0)
            {
                std::cout << "Writing to log succesfully" << std::endl;
            }
            else
            {
                std::cout << "Writing failed! Errno: " << errno << ": ";
                std::cout << strerror(errno) << std::endl;
            }
        }
        
        if (ControlDataToPython == 1) // only needs to write if the command is given
        {
            pthread_mutex_lock(&mutex2);
            freopen(pipe_control_data_to_python, "w", stdout);
            control_data_pipe.write_struct_control_data(gl_control_data_ptr);
            fclose(stdout);
            pthread_mutex_unlock(&mutex2);
            ControlDataToPython = 0;
        }
         
        usleep(CONTROL_DATA_WRITE_CYCLE_TIME);
    }
}

void *ThreadWriteUserPowerData(void *ptr)
{
    /*! \fn void *ThreadWritePipe(void *ptr)
     \brief This is a thread to open the pipe, write data, and closes it every 10000 microseconds
     \param *ptr for making thread
     */
    int FifoB;
    char tempString[1000];
    while (1)
    {
        if (UserPowerToLogfile == 1) // only needs to write if the command is given
        {
            //strcpy(tempString, OutputString); // copies data from WriteString function
            pthread_mutex_lock(&mutex3);
         
            user_power_logger.write_struct_user_power(gl_power_input_ptr, gl_power_output_ptr, gl_user_input_ptr);
            UserPowerToLogfile = 0;
           
            pthread_mutex_unlock(&mutex3);
            if (errno == 0)
            {
                std::cout << "Fifo B opening/write succesfully" << std::endl;
            }
            else
            {
                std::cout << "Writing/opening Fifo B failed! Errno: " << errno << ": ";
                std::cout << strerror(errno) << std::endl;
            }
        }
        if (UserPowerToPython == 1) // only needs to write if the command is given
        {
            pthread_mutex_lock(&mutex3);
            freopen(pipe_user_power_to_python, "w", stdout);
            user_power_pipe.write_struct_user_power(gl_power_input_ptr, gl_power_output_ptr, gl_user_input_ptr);
            fclose(stdout);
            UserPowerToPython = 0;
           
            pthread_mutex_unlock(&mutex3);
        }
        usleep(USER_POWER_WRITE_CYCLE_TIME);
    }
}

void *ThreadWriteTelemInputData(void *ptr)
{
    /*! \fn void *ThreadWritePipe(void *ptr)
     \brief This is a thread to open the pipe, write data, and closes it every 10000 microseconds
     \param *ptr for making thread
     */
    int FifoB;
    char tempString[1000];
    while (1)
    {
        if (TelemInputToLogfile == 1) // only needs to write if the command is given
        {
            //strcpy(tempString, OutputString); // copies data from WriteString function
            pthread_mutex_lock(&mutex4);
         
           
            telem_input_logger.write_struct_telemetry_input(gl_telemetry_input_ptr);
            //fclose(stdout);
            TelemInputToLogfile = 0;
           
            pthread_mutex_unlock(&mutex4);
            if (errno == 0)
            {
                std::cout << "Fifo B opening/write succesfully" << std::endl;
            }
            else
            {
                std::cout << "Writing/opening Fifo B failed! Errno: " << errno << ": ";
                std::cout << strerror(errno) << std::endl;
            }
        }
        if (TelemInputToPython == 1) // only needs to write if the command is given
        {
            pthread_mutex_lock(&mutex4);
            freopen(pipe_telem_input_to_python, "w", stdout);
            telem_input_pipe.write_struct_telemetry_input(gl_telemetry_input_ptr);
            fclose(stdout);
            TelemInputToPython = 0;
           
            pthread_mutex_unlock(&mutex4);
        }
        usleep(TELEM_INPUT_WRITE_CYCLE_TIME);
    }
}
 

inline bool exist_test(const char * name_char)
{
    std::string name = name_char;
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

