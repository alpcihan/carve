#pragma once

#include <iostream>

#define CRV_ERROR(message)  std::cerr << "[ERROR]: " << message << "\n";

#ifndef CRV_DISABLE_INFO_LOGS

#define CRV_INFO(message)   std::cout << "[INFO]: " << message << "\n";

#else

#define CRV_INFO(message)

#endif