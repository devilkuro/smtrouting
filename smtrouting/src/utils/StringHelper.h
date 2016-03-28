/*
 * StringHelper.h
 *
 *  Created on: Oct 4, 2015
 *      Author: Fanjing-R830
 */

#ifndef STRINGHELPER_H_
#define STRINGHELPER_H_
#include <sstream>
namespace Fanjing {

/*
 *
 */
using namespace std;
class StringHelper {
private:
    static stringstream ss;

public:
    static int str2int(string s);
    static int char2int(char* s);
    static double str2dbl(string s);
    static double char2dbl(char* s);
    static string int2str(int i);
    static string dbl2str(double d, int precision = -1);
};

} /* namespace Fanjing */
#endif /* STRINGHELPER_H_ */
