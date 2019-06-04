/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   easy_debugging.hpp
 * Author: Sander Oosterveld
 *
 * Created on April 14, 2019, 1:07 PM
 */

#ifndef EASY_DEBUGGING_HPP
#define EASY_DEBUGGING_HPP

#include <type_traits>
#include <string>
#include <iostream>
#include <typeinfo>
#include <stdint.h>
#include <stdlib.h>


#define RST   "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define BOLD  "\e[1m"

#define ERR(x) KRED x RST
#define WARN(x) KYEL x RST
#define INFO(x) KBLU x RST
#define OK(x)   KGRN x RST

#define INFO_STR "INFO"
#define WARN_STR "WARNING"
#define ERR_STR "ERROR"
#define DEBUG_STR "DEBUG"
#define OK_STR "OK"

#define DEBUG_INFO  true
#define DEBUG_WARN  true
#define DEBUG_ERR  true
#define DEBUG_DEBUG  true
#define DEBUG_OK  true

//#define PRETTY_PRINT

namespace debug{


class Debug
{
  static const bool DEBUG_MIO = false;
  
  
 public:
  Debug(const std::string &funcName, const std::string &color, const std::string &type, const bool debugging)
  {
    DEBUG_ON = debugging;
    if(DEBUG_ON){
      std::cout<<color<<type<<"( "<<funcName<< " ): ";
    } else {
      std::cout<<"";
    }
  }
  
  template <class T>
  Debug &operator<<(const T &v)
  {
    if(DEBUG_ON){
      if(typeid(v)==typeid(uint8_t)){
        std::cout<<std::hex<<std::showbase<<v<<std::dec;
        //printf("0x%x",(v));
      } else if(typeid(v)==typeid(short int)){
        std::cout<<std::hex<<std::showbase<<v<<std::dec;
        //printf("0x%x",(v));
      } else{ 
        std::cout<<v;
      }
    } else{
      std::cout<<"";
    }
    return *this;
  }

  
  ~Debug(){
    if(DEBUG_ON){
      std::cout<<RST<<"\n";
    }
    
    
  }
  
  template<class M>
  Debug &printArray(M arr[], int length){
    for(int i = 0; i<length; i++){
      std::cout<<std::hex<<std::showbase<<(int)arr[i]<<" "<<std::dec;
    }
    return *this;
  }
  
  
 private:
  bool DEBUG_ON = false;
};

//typedef  debug::Debug(__FUNCTION__, "HELLO: ", "INFO", true) MY_INFO;
}
#ifdef PRETTY_PRINT
#define M_INFO debug::Debug(__PRETTY_FUNCTION__, KBLU, INFO_STR, DEBUG_INFO)
#define M_WARN debug::Debug(__PRETTY_FUNCTION__, KYEL, WARN_STR, DEBUG_WARN)
#define M_ERR debug::Debug(__PRETTY_FUNCTION__, KRED, ERR_STR, DEBUG_ERR)
#define M_DEBUG debug::Debug(__PRETTY_FUNCTION__, KWHT, DEBUG_STR, DEBUG_DEBUG)
#define M_OK debug::Debug(__PRETTY_FUNCTION__, KGRN, OK_STR, DEBUG_OK)
#else
#define M_INFO debug::Debug(__FUNCTION__, KBLU, INFO_STR, DEBUG_INFO)
#define M_WARN debug::Debug(__FUNCTION__, KYEL, WARN_STR, DEBUG_WARN)
#define M_ERR debug::Debug(__FUNCTION__, KRED, ERR_STR, DEBUG_ERR)
#define M_DEBUG debug::Debug(__FUNCTION__, KWHT, DEBUG_STR, DEBUG_DEBUG)
#define M_OK debug::Debug(__FUNCTION__, KGRN, OK_STR, DEBUG_OK)
#endif

#endif /* EASY_DEBUGGING_HPP */

