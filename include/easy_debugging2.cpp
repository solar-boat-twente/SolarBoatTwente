/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "easy_debugging.hpp"

using namespace debug;
namespace debug{
template<typename T>
int printArray(T& printed_array, short int length){
  std::cout<<"test";
};
}
/*
template<class T>
void debug::printArray(T printed_array[], short int length){
  if(DEBUG_INFO){
    for(int i = 0; i<length; i++){
      std::cout<<BOLD<<(int)printed_array[i]<<" ";
    }
    std::cout<<RST<<std::endl;
  }
}*/
