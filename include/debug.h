#ifndef LP_MP_DEBUG_HXX
#define LP_MP_DEBUG_HXX

#include <string>
#include <typeinfo>

namespace LP_MP {
namespace DBG {

template<class T>
std::string demangled_name(T &object)
{
	int status;
	char *demangled = abi::__cxa_demangle(typeid(object).name(), 0, 0, &status);
	if (status != 0)
		throw std::runtime_error("Demangling failed.");
	std::string result(demangled);
	free(demangled);
	return result;
}

} // end namespace debug
} // end namespace LP_MP

#endif
