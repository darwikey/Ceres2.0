#ifndef CLI_H
#define CLI_H


#define CLI_DEL_CHAR  127
#define CLI_DELIMITER ' '
#define CLI_BUFFER_SIZE 128

class CommandLineInterface
{
public:
	static CommandLineInterface Instance;
	void Init();
	void Task();
	void PrintHelp();

private:
	struct Command
	{
		const char *cmd;
		const char *help;
		typedef void (*FunctionDecl)(const char**, int);
		FunctionDecl function;
	};

	Command m_Command[128];

	char m_Buffer[CLI_BUFFER_SIZE];
	int m_CurPos = 0;
};
#endif /* CLI_H */
