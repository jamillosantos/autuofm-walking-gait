/**
 * @author J. Santos <jamillo@gmail.com>
 * @date October 06, 2016
 */

#ifndef WALKING_CONSTS_H
#define WALKING_CONSTS_H

#include <math.h>

#define M_2xPIl (M_PIl * 2)
#define M_32_PIl (3.0 * M_PI_2l)

#define RAD2DEG (180.0/M_PI_2)


#define VERBOSE_HEADER "[VERBOSE] " << __FILE__ << ":" << __LINE__ << ") "
#define VERBOSE(x) std::cout << VERBOSE_HEADER << x << std::endl
#define VERBOSENEL(x) std::cout << VERBOSE_HEADER << x
#define VERBOSEDATA(d, s) Interface::dumpData(d, s)
#define VERBOSEB(x) std::cout << x << std::endl
#define VERBOSEBNEL(x) std::cout << x

/*
#define VERBOSE_HEADER
#define VERBOSE(x)
#define VERBOSENEL(x)
#define VERBOSEDATA(d, s)
#define VERBOSEB(x)
#define VERBOSEBNEL(x)
*/

#define ERROR_HEADER "[ERROR] " << __FILE__ << ":" << __LINE__ << ") "
#define ERROR(x) std::cerr << ERROR_HEADER << x << std::endl
#define ERRORNEL(x) std::cerr << ERROR_HEADER << x
#define ERRORB(x) std::cerr << x << std::endl
#define ERRORBNEL(x) std::cerr << x

#endif //WALKING_CONSTS_H
