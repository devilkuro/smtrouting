/*
 * StringHelper.cc
 *
 *  Created on: Oct 4, 2015
 *      Author: Fanjing-R830
 */

#include "StringHelper.h"

namespace Fanjing {

stringstream StringHelper::ss;

int StringHelper::str2int(string s) {
    int i;
    ss << s;
    ss >> i;
    ss.str("");
    return i;
}

int StringHelper::char2int(char* s) {
    return str2int(string(s));
}

double StringHelper::str2dbl(string s) {
    double d;
    ss << s;
    ss >> d;
    ss.str("");
    return d;
}

double StringHelper::char2dbl(char* s) {
    return str2dbl(string(s));
}

string StringHelper::int2str(int i) {
    return dbl2str(i);
}

string StringHelper::dbl2str(double d, int precision) {
    string s;
    if (precision != -1) {
        ss << fixed;
        ss.precision(precision);
        ss << d;
        s = ss.str();
        ss.str("");
        ss.unsetf(ios_base::floatfield);
        // reset to default precision
        ss.precision(6);
    } else {
        ss << d;
        s = ss.str();
        ss.str("");
    }
    return s;
}

string StringHelper::convertStrToFileName(string str) {
    // '/','\','*','?','<','>',':','|',',','"'
    string invalidChar = "/\\*?<>:|,\"";
    string strChar = "_";
    for (unsigned int i = 0; i < invalidChar.length(); ++i) {
        for (unsigned int strPos = 0; strPos < str.length();) {
            // ...strPos,+1,+2...
            std::size_t offset = str.find(invalidChar.at(i), strPos);
            if (offset != string::npos && offset < str.length()) {
                string re = strChar + int2str(i);
                str.replace(offset, 1, re);
                strPos = offset + re.length();
            } else {
                break;
            }
        }
    }
    return str;
}

} /* namespace Fanjing */

