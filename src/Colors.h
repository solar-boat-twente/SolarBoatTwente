/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Colors.h
 * Author: Sander Oosterveld
 *
 * Created on April 13, 2019, 1:36 PM
 */

#ifndef COLORS_H
#define COLORS_H
#include <string>
namespace Color{

#define RST   "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define ERR(x) KRED x RST
#define WARN(x) KYEL x RST
#define INFO(x) KBLU x RST
#define OK(x)   KGRN x RST



}

#endif /* COLORS_H */

