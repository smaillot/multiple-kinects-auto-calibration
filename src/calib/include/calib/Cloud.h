#include <string>

class Cloud
{
private:
	std::string pub_name
	std::string sub_name
public:
	Cloud(std::string sub_name, std::string pub_name);
	virtual ~Cloud();
};