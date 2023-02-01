#pragma once

#include <iostream>

#ifdef CRV_ENABLE_ERROR_LOGS

#define CRV_ERROR(message)  std::cerr << "[ERROR]: " << message << "\n";

#else

#define CRV_ERROR(message)

#endif

#ifdef CRV_ENABLE_INFO_LOGS

#define CRV_INFO(message)   std::cout << "[INFO]: " << message << "\n";

#else

#define CRV_INFO(message)

#endif