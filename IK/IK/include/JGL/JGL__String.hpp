//
//  JGL__String.hpp
//  TextilePatternGeneration
//
//  Created by Hyun Joon Shin on 2020/10/05.
//  Copyright © 2020 Hyun Joon Shin. All rights reserved.
//

#ifndef JGL__String_hpp
#define JGL__String_hpp

#include <string>
#include <sstream>

namespace JGL {
/// \brief The anything to string converter.
///
/// Converting anything to std::string.
/// Note that while the std::to_string leaves tailing zeros for floating point values,
/// this function truncate the tailing zeros.
/// @param t The value to be converted into std::string.
/// @return The string representing the input data.
template<typename T> std::string to_str (const T & t) {
	std::ostringstream os;
	os.precision(7);
	os << t;
	return os.str ();
}

/// \brief UFT32 to UTF8 converter.
///
/// Converting a unicode character represented with UTF32 into UFT8 string.
/// @param c UTF32 code point.
/// @return std::string in UTF8.
extern std::string to_utf8str(char32_t c);

} // namespace JGL

#endif /* JGL__String_hpp */
