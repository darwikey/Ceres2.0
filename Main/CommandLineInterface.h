#ifndef CLI_H
#define CLI_H


#define CLI_DEL_CHAR  127
#define CLI_ESC_CHAR  27
#define CLI_HISTORY_CHAR 178
#define CLI_DELIMITER ' '
#define CLI_BUFFER_SIZE 128

#define CLI_ARG_LENGTH 20
#define CLI_MAX_ARG 5

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
		typedef void (*FunctionDecl)(const char [CLI_MAX_ARG][CLI_ARG_LENGTH], int);
		FunctionDecl function;
	};

	Command m_Command[128];

	char m_Buffer[CLI_BUFFER_SIZE];
	char m_History[CLI_BUFFER_SIZE] = {0};
	int m_CurPos = 0;
};
#endif /* CLI_H */
