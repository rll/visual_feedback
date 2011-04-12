#include <exception>
#include <string>

namespace CVGreyc {
namespace Exceptions {

class Exception : public std::exception {
	public:
	typedef std::string message_t;
	private:
	message_t _message;
	public:
	Exception() throw() {};
	Exception(const char* message):_message(message){};
	virtual ~Exception() throw() {};
	virtual const char* what() const throw() {return _message.c_str();}
};
	
}		
}
