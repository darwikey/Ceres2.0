#ifndef CLI_H
#define CLI_H


#define CLI_DEL_CHAR  127
#define CLI_DELIMITER ' '
#define CLI_BUFFER_SIZE 128

class CommandLineInterface
{
public:
	static CommandLineInterface Instance;
	void Task();

private:
	char m_Buffer[CLI_BUFFER_SIZE];
	int m_CurPos = 0;
};
#endif /* CLI_H */
